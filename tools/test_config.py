"""Check that every tuned number lives in config.py, and only there.

    python3 tools/test_config.py

Two failures this guards against, both of which had already happened:

  A tuned value with a second home. LineFollower defaulted to speed 60 and
  steer 40 while config.py said 50 and 20. Whichever was edited, the other
  stayed, and tools/tune.py measured a steer of 40 straying 17.6 mm on a
  tape 10 mm wide. Drive.ROTATE_SCALE was worse: measured on the table
  during M1 and then buried in a driver, where nobody tuning the robot
  would think to look.

  A tuned value with no home at all - a constant left in config.py that
  nothing reads, so adjusting it does nothing and the real figure is
  somewhere else.

So: no constructor in sw/ may default a numeric parameter, and no tuning
constant in config.py may go unread.
"""

import ast
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(HERE, "picostub"))
sys.path.insert(0, os.path.join(REPO, "sw"))

import config                                            # noqa: E402

failures = []


def check(what, got, want):
    ok = got == want
    if not ok:
        failures.append("{} (got {!r}, wanted {!r})".format(what, got, want))
    print("  {}  {:<50} {!r}".format("ok  " if ok else "FAIL", what, got))


# Values small enough to be structural rather than tuned: a zero default, a
# unit step, a sign. Anything bigger is a measurement and belongs in config.
def is_tunable(value):
    return (isinstance(value, (int, float)) and not isinstance(value, bool)
            and abs(value) > 1)


def sources(root, skip_dirs=("libs",), skip_files=("config.py",)):
    for dirpath, dirnames, filenames in os.walk(root):
        dirnames[:] = [d for d in dirnames if d not in skip_dirs]
        for fn in sorted(filenames):
            if fn.endswith(".py") and fn not in skip_files \
                    and not fn.startswith("test_"):
                yield os.path.join(dirpath, fn)


def numeric_defaults():
    """Every function in sw/ that defaults a parameter to a tuned number."""
    found = []
    for path in sources(os.path.join(REPO, "sw")):
        tree = ast.parse(open(path).read())
        for node in ast.walk(tree):
            if not isinstance(node, ast.FunctionDef):
                continue
            defaults = list(node.args.defaults)
            if not defaults:
                continue
            names = [a.arg for a in node.args.args][-len(defaults):]
            for name, default in zip(names, defaults):
                if isinstance(default, ast.Constant) \
                        and is_tunable(default.value):
                    found.append("{}:{} {}({}={!r})".format(
                        os.path.relpath(path, REPO), node.lineno,
                        node.name, name, default.value))
    return found


def module_constants():
    """Tuned numbers assigned at module or class level in sw/.

    Only the top level and class bodies - walking the whole tree would
    catch ordinary local variables, such as the clamp inside Motor.drive().
    """
    found = []
    for path in sources(os.path.join(REPO, "sw")):
        tree = ast.parse(open(path).read())
        statements = list(tree.body)
        for node in tree.body:
            if isinstance(node, ast.ClassDef):
                statements.extend(node.body)
        for node in statements:
            if not isinstance(node, ast.Assign):
                continue
            for target in node.targets:
                if not isinstance(target, ast.Name):
                    continue
                if not isinstance(node.value, ast.Constant):
                    continue
                if not is_tunable(node.value.value):
                    continue
                # A full-scale or private hardware constant is a fact about
                # the chip, not something anyone tunes.
                if target.id.startswith("_"):
                    continue
                found.append("{}:{} {} = {!r}".format(
                    os.path.relpath(path, REPO), node.lineno,
                    target.id, node.value.value))
    return found


def unread_constants():
    """Tuning constants in config.py that nothing anywhere reads."""
    names = [n for n in dir(config)
             if n.isupper() and not n.startswith("_")]
    text = ""
    for root in ("sw", "tools"):
        for dirpath, dirnames, filenames in os.walk(os.path.join(REPO, root)):
            dirnames[:] = [d for d in dirnames if d not in ("libs", "out")]
            for fn in filenames:
                if fn.endswith(".py") and fn != "config.py":
                    text += open(os.path.join(dirpath, fn)).read()
    return sorted(n for n in names if n not in text)


def main():
    print("no tuned number is defaulted inside a driver")
    check("constructors with a numeric default", numeric_defaults(), [])

    print("\nno tuned number is defined at module level in sw/")
    # Two that are not tuning values:
    #   BAYS_PER_RACK is a fact about the table, and test_map.py already
    #   checks it against the map - six bays per shelf, twenty-four total.
    #   RIGHT is a bit flag, so that LEFT | RIGHT gives BOTH.
    allowed = {"sw/nav/destination.py:53 BAYS_PER_RACK = 6",
               "sw/nav/junction.py:45 RIGHT = 2"}
    check("module constants outside config.py",
          [c for c in module_constants() if c not in allowed], [])

    print("\nevery constant in config.py is read somewhere")
    check("unread constants", unread_constants(), [])

    print("\nthe scan window can reach a verdict at all")
    check("RANGER_WINDOW_SAMPLES >= RANGER_CONFIRM_READS",
          config.RANGER_WINDOW_SAMPLES >= config.RANGER_CONFIRM_READS, True)
    # Both verdicts need confirm_reads of their own, so a window shorter
    # than twice that can never report EMPTY once anything near is in it.
    check("the window can hold both verdicts' worth of samples",
          config.RANGER_WINDOW_SAMPLES >= config.RANGER_CONFIRM_READS, True)
    check("RANGER_POLL_MS exceeds one VL53L0X measurement (~30 ms)",
          config.RANGER_POLL_MS >= 30, True)

    print("\na turn has time to find the line again")
    for name, over, commit in (
            ("turn", config.TURN_DRIVE_OVER_MS, config.TURN_COMMIT_MS),
            ("uturn", config.UTURN_DRIVE_OVER_MS, config.UTURN_COMMIT_MS)):
        check("TURN_TIMEOUT_MS exceeds the {} phases ({} ms)".format(
            name, over + commit), config.TURN_TIMEOUT_MS > over + commit, True)

    print("\nthe clock leaves room for a whole cycle")
    check("CYCLE_RESERVE_MS fits inside RUN_LIMIT_MS",
          0 < config.CYCLE_RESERVE_MS < config.RUN_LIMIT_MS, True)

    print("\nno pin is claimed twice")
    pins = {}
    for name in dir(config):
        if name.endswith("_PIN") and isinstance(getattr(config, name), int):
            pins.setdefault(getattr(config, name), []).append(name)
    for side, spec in config.RANGER_PINS.items():
        pins.setdefault(spec["pin"], []).append("RANGER_" + side.upper())
        for key in ("sda", "scl"):
            pins.setdefault(spec[key], []).append(
                "RANGER_{}_{}".format(side.upper(), key.upper()))
    clashes = {no: who for no, who in pins.items() if len(who) > 1}
    # The turn trackers are ignored while unfitted, so they may share.
    check("pins claimed by more than one function", clashes, {})

    print("\nvalues still to be measured on the table")
    pending = []
    for line in open(os.path.join(REPO, "sw", "config.py")):
        if "CHECK" not in line and "MEASURE" not in line:
            continue
        if "=" not in line or line.lstrip().startswith("#"):
            continue            # prose, not an assignment
        name = line.split("=")[0].strip()
        if name.isupper():
            pending.append(name)
    print("  {} constants marked CHECK: {}".format(
        len(pending), ", ".join(pending)))

    print()
    if failures:
        print("{} FAILED".format(len(failures)))
        for f in failures:
            print("  -", f)
        return 1
    print("all checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
