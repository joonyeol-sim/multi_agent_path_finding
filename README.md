Installation
===============

You can install the latest version of Multi-Agent Path Planning from the [Python Package Index (PyPI)](https://pypi.org/project/multi-agent-path-finding/):

```shell
pip install multi-agent-path-finding
```

Alternatively, you can clone the repository and install the package locally:

```shell
git clone git@github.com:joonyeolsim/multi_agent_path_finding.git
cd multi_agent_path_finding
pip install -e .
```

Usage
===============

We provide the examples of usage of the algorithms in the following sections.
If you want to run the examples, please install the repository from the source.
Below commands are executed in the examples directory.

Space Time Astar Example
---------------
```bash
python3 stastar/example.py -i ../configs/stastar/random_input.yaml -o output.yaml
```
- -i: input file path
- -o: output file path

Space Time Astar Epsilon Example
---------------
```bash
python3 stastar_epsilon/example.py -i ../configs/stastar_epsilon/random_input.yaml -o output.yaml -w 1.1
```
- -i: input file path
- -o: output file path
- -w: suboptimality bound

Conflict Based Search Example
---------------
```bash
python3 cbs/example.py -i ../configs/cbs/random_input.yaml -o output.yaml
```
- -i: input file path
- -o: output file path

Enhanced Conflict Based Search Example
---------------
```bash
python3 ecbs/example.py -i ../configs/ecbs/random_input.yaml -o output.yaml -w 1.1
```
- -i: input file path
- -o: output file path
- -w: suboptimality bound

Proceeding
===============
- [x] Space Time Astar
- [x] Space Time Astar Epsilon
- [x] Conflict Based Search
- [x] Enhanced Conflict Based Search
- [ ] Conflict Based Search Task Assignment
- [ ] Enhanced Conflict Based Search Task Assignment
- [ ] Prioritized Safe-Interval Path Planning
- [ ] Conflict Based Search with Bypass

References
===============
[1] Sharon, Guni, et al. "Conflict-based search for optimal multi-agent pathfinding." Artificial Intelligence 219 (2015): 40-66.

[2] Silver, David. "Cooperative pathfinding." Proceedings of the aaai conference on artificial intelligence and interactive digital entertainment. Vol. 1. No. 1. 2005.

[3] Barer, Max, et al. "Suboptimal variants of the conflict-based search algorithm for the multi-agent pathfinding problem." Proceedings of the International Symposium on Combinatorial Search. Vol. 5. No. 1. 2014.
