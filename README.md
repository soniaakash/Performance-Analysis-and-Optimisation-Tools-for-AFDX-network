# Performance Analysis and Optimisation Tools for AFDX network

This is a group of tools developed for analysis and optimisation of AFDX network.
I wrote this code during the work of my PhD thesis which can be found at http://www.theses.fr/

It is basically an implementation of Network Calculus approach in the following contexts:

1) Worst-case end-to-end transmission delay analysis in AFDX network
    It includes following scheduling policies:
        DRR
        WRR
        FIFO
        SPQ
2) Tuning of QoS based on: 
        DRR and WRR scheduling

The code is written in C++ using the Qt library. The choice of Qt library is purely personal and the code can be easily modified to remove its Qt dependence.

The project is divided into three main parts:
1) AFDX/
    It is an implementation of AFDX network configuration. It includes the basic definition of flows and network nodes.
2) NetworkCalculus/
    It is an implementation of Network Calculus approach. It includes redefinition of flows and network nodes and definitions of WCTT and Qos Tuning with different scheduling policies.
3) SharedData/
    It is a piece of code shared globally to provide the basic tools for: floating-point-precision management, data-type conversion, print results ...

I hope this project will be helpful to my fellow researchers!

Disclaimer: This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation. See the License file for more information. This program was developed for research purpose only and is available to the communtity for free as an open source project, I take no liability for damages resulting from using the projects.

Keywords : Network Calculus, AFDX, WCTT Analysis, QoS Tuning, DRR, WRR, SPQ, Real-Time Performance Analysis


If you like this project and if it was helpful in your research, please dont forget to cite.
