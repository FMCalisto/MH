# Introduction

The objective of this assignment is to design an optimized cache architecture targeted for a specific
embedded application: a video encoder. This architecture should be optimized in order to (i) minimize
the monetary cost of the system, and (ii) improve performance by reducing the overall cache miss penalty
time associated to the execution of the video encoding algorithm.

In the rest of this section, we briefly describe the targeted application (Section 1.1), present
the requirements and restrictions for the memory hierarchy architecture (Section 1.2), and introduce
DineroIV [1], the cache simulator that will be adopted to assess the performance of the memory hierarchy
architecture (Section 1.3). In Section 2, we guide the students in designing a cache structure that
best matches the requirements of the considered application program.


## Targeted Application

The application whose performance we wish to improve is a video encoder, more specifically, its mo-
tion estimation algorithm. This algorithm is usually the most computationally intensive operation of any
MPEG-x video encoding system. It aims to improve the efficiency of video compression by exploiting
both the temporal and spacial redundancy within an image sequence. To achieve this, the motion
estimation algorithm reduces the residual information between contiguous frames by tracking and compensating
movement using previously encoded images.

For this assignment, we will work with a real implementation of a motion estimation algorithm. The
C source code file of a real implementation of this algorithm can be obtained at the course webpage
(file motion_estimation.c). However, it is not necessary to understand the inner workings of this
algorithm. For the interested reader, we explain the details of motion estimation in Appendix A.


## Memory Hierarchy Architecture

To improve the performance of the video encoder, it is necessary to design a dedicated memory hierarchy
architecture taking into account the following set of requirements and restrictions.

The memory hierarchy must consist of a dedicated SDRAM memory called frame memory and a set
of on-chip and off-chip SRAM cache memories. The frame memory is responsible for accommodating
the 8-bit pixel values of the video frames that will be considered for the motion estimation operation. To
accommodate the video frames, this memory must have a capacity of 128 kBytes. Each pixel value is
stored in its own memory address using a raster format (from left to right and from top to bottom). The
cache memories interconnect the SDRAM memory and the motion estimation processor. Due to current
market restrictions, the overall price of the desired memory hierarchy cannot exceed e 0.012.

To design an optimal memory architecture, one should investigate a configuration that provides the
best performance benefits for the targeted application while abiding by the price restrictions stated above.
Thus, a configuration is optimal when it produces the lowest cost value for the following normalized cost
function. This function weighs both these factors by multiplying the Mean Access Time by the Cost of
the memory hierarchy (Frame memory + Caches).

```
Cost Function = Mean Access Time [ns] × Price of the memory hierarchy [€ × 10^−3]

```


## Simulation Environment

To evaluate the performance of a memory architecture, we recommend using DineroIV [1], a trace-driven
cache simulator. DineroIV is capable of simulating a complete memory hierarchy consisting of various
caches interconnected between the processor and the primary memory.

To perform a simulation, DineroIV takes as inputs a parameter set and a trace file. The parameter
set specifies the characteristics of the caches to be simulated, for example the cache size, the block size,
and the number of associativity ways. The trace file contains all memory references accessed by the
program under evaluation. These traces specify whether a memory reference is an instruction fetch, a
data read (LOAD), or a data write (STORE). For each reference, DineroIV simulates the behavior of the
specified cache configuration and generates hits and misses as appropriate. At the end of the simulation,
it produces a set of statistics which summarizes the performance of the simulation, including number of
references, misses and memory traffic generated by the processor.
To show a concrete usage example, consider the following execution of DineroIV, which is launched
from the command line:

```
dineroIV -l1-usize 8k -l1-ubsize 8
         -l2-usize 128k -l2-ubsize 16 < trace.xdin
```

In this example, DineroIV simulates a caching architecture comprising a cache L1 and a cache L2.

The cache L1 has 8 kBytes and a block size of 8 bytes. The cache L2 has 128 kBytes and a block size of
16 bytes. The trace file that contains the read/write memory access patterns is provided in file trace.xdin.
DineroIV is extensively described in its user manual [3]. Nevertheless, we summarize the most
relevant parameters for the purpose of this assignment. The basic command to run DineroIV is:

```
dineroIV [options] < tracefile
```

where the most relevant parameters are:

`−lN−Tsize P`  - Sets the cache size of the specified level N cache to P bytes
`−lN−Tbsize P` - Sets the block size of the specified level N cache to P bytes
`−lN−Tassoc U` - Sets the associativity of the specified level N cache to U
`−lN−Tccc`     - Computes Compulsory/Capacity/Conflict miss rates for the specified level N
                 cache

where:

`T` - is the cache type (u=unified, i=instruction, d=data);
`N` - is the cache level (1  N), where level 1 is closest to the processor;
`U` - is an unsigned decimal integer;
`P` - is like U, but must be a power of 2, with an optional scaling suffix character (one of kKmMgG,
      for multiplication by 0x400, 0x100000, or 0x40000000).
      
      
Since DineroIV requires a trace file containing the sequence of memory accesses to be performed, for
this assignment it is necessary to generate such a trace file for the motion estimation algorithm described
in Section 1.1. To this end, the supplied implementation (program motion_estimation.c) includes
two special functions (`frame_memory_read()` and `frame_memory_write()`) for creating the trace
file (`trace.log`). This information will then enable the simulation of several cache configurations in
order to obtain the one that offers the best performance levels within an acceptable cost.
