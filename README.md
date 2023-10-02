# 802.11 REINRATE

This repository contains the code associated with the paper: 
"A Reinforcement Learning Approach to Wi-Fi Rate Adaptation Using the REINFORCE Algorithm."

## Introduction

In this work, we introduce a rate adaptation in 802.11 wireless networks using the REINFORCE reinforcement learning algorithm. Our strategy considers a broader set of observations. This includes received signal strength, contention window size, current Modulation and Coding Scheme (MCS), and throughput. By leveraging this rich set of data, our approach can adapt more granularly to changing network conditions, ultimately leading to optimized network throughput.

## Prerequisites

1. **NS3-AI**:
   - Make sure you have NS3-AI installed.
   - For installation guidelines, refer to [NS3-AI Installation Guide](https://github.com/hust-diangroup/ns3-ai).

2. **NS3**:
   - The code has been tested on NS3 version 3.36.

## Installation & Setup

1. Clone this repository into your NS3's `scratch` directory:
   ```bash
   git clone <repository-link> <path-of-ns3>/scratch
2. Navigate to the example directory and run the provided Python script:
  `cd scratch/reinrate'
  'python ns3ai.py'

## Cite Our Work
If you find this work useful in your research or projects, please consider citing our paper:

@inproceedings{ReinRate2024,
title={A Reinforcement Learning Approach to Wi-Fi Rate Adaptation Using the REINFORCE Algorithm},
author={Ye, Tao and Wee-lum, Tan},
booktitle={Proceedings of IEEE Wireless Communications and Networking Conference 2024},
year={2024},
}
