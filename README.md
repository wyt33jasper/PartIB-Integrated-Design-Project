# Part IB Integrated Design Project (IDP)

AGV for package pickup and delivery — M1 2026.

## 📖 [**Software documentation →**](DOCUMENTATION.md)

Start there. It covers the pin map, how to flash the Pico, the algorithm
flowchart, a walkthrough of `main.py`, every function in `sw/` and `tools/`,
the configuration reference and what is still to be measured.

| | |
|---|---|
| [`sw/`](sw) | The firmware. `main.py` runs automatically at power-up |
| [`sw/config.py`](sw/config.py) | Every pin number and every tuned constant, in one place |
| [`tools/`](tools) | Host-side checks and simulations — run on a PC, never uploaded |
| [`docs/navigation.md`](docs/navigation.md) | Why the navigation is designed the way it is |
| [`docs/map-survey.md`](docs/map-survey.md) | Survey sheet to fill in at the table |

```bash
python3 tools/test_config.py     # pins, limits, and where tuned numbers live
python3 tools/test_map.py        # the map is consistent; every route exists
python3 tools/test_mission.py    # the planner, against the real map
python3 tools/simulate_run.py    # boots sw/main.py and drives a whole run
```

## License and copyright

All material is copyright of the original author (see the git commit).

Some material is redistributed under licenses from the original codebase,
please refer to the licenses associated with, for example, special
libraries.

All text is made available under the Creative Commons
Attribution-ShareAlike 4.0 International Public License
(https://creativecommons.org/licenses/by-sa/4.0/legalcode).

All computer code is released under the MIT license.

The MIT License (MIT)

Permission is hereby granted, free of charge, to any person obtaining
a copy of this software and associated documentation files (the
"Software"), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish,
distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to
the following conditions:

The above copyright notice and this permission notice shall be
included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.