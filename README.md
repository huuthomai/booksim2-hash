BookSim Interconnection Network Simulator
=========================================

BookSim is a cycle-accurate interconnection network simulator.
Originally developed for and introduced with the [Principles and Practices of Interconnection Networks](http://cva.stanford.edu/books/ppin/) book, its functionality has since been continuously extended.
The current major release, BookSim 2.0, supports a wide range of topologies such as mesh, torus and flattened butterfly networks, provides diverse routing algorithms and includes numerous options for customizing the network's router microarchitecture.

---

If you use BookSim in your research, we would appreciate the following citation in any publications to which it has contributed:

Nan Jiang, Daniel U. Becker, George Michelogiannakis, James Balfour, Brian Towles, John Kim and William J. Dally. A Detailed and Flexible Cycle-Accurate Network-on-Chip Simulator. In *Proceedings of the 2013 IEEE International Symposium on Performance Analysis of Systems and Software*, 2013.


-------

## Booksim_THO
Tho Mai - CSNL KAIST (2023)

Some main modifications/features:
1. Simulation modes/configurations
    - Message-based simulation
	- Batch mode with Injection rate
	- Compute-Memory configuration
	- Faulty network configuration
	- Hotspot configuration
2. Metrics
	- Re-ordering latency and re-ordering buffer occupancy (credit: Hans)
	- Channel utilization
	- Blocking time
3. Topologies
	- Fat-tree with x2 bandwidth (2-level only)
4. Routing 
	- Hash-based routings for Fat-tree
	- Flit counter 
5. Traffics
	- Worst/Best case traffic for modulo routings
	- PERM(n) (1 source n destinations)
	- Asymmetric traffics
...
