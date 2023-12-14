# Dynamic Neural Curiosity

## Description

This repository gathers all necessary packages and source code to run the dynamic neural curiosity architecture.

<div style="display:flex">
     <div style="flex:1;padding-right:10px;">
       <img src="https://github.com/rouzinho/Dynamic-Neural-Curiosity/assets/10597250/d95a08f5-bc72-45d9-86d8-2d89cb74d05c" width="395"/>
        <img src="https://github.com/rouzinho/Dynamic-Neural-Curiosity/assets/10597250/bf68ae0f-73ec-431c-b334-4f51edb7af49" width="372"/>
     </div>
</div>

The purpose of the experiment is to root attention and curiosity together by taking inspiration from neuroscience, more especially about the role of the Locus Coeruleus. The architecture is a robotics implementation where a robot arm continuously switch between goal discovery (exploration) and learning (exploitation). The robot first discovers new goals through bottom-up attention, then select and monitor the learning of these goals through dynamic curiosity. We call here dynamic curiosity the evolution of the forward model error through a learning progress. The complete process is modelled with Dynamic Neural Fields, leveraging interesting properties regarding the selection of goals as well as the learning dynamics that can occur. You can find complete results in the paper.

## Results

![test](https://github.com/rouzinho/Dynamic-Neural-Curiosity/assets/10597250/e9fd152f-51fc-49b4-8259-b81a09694f60)
