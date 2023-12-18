# RosHebbian

Plugin that formalizes datas coming for the neural fields before sending them for learning. The module takes 3 inputs : current goal, explore and exploit. If the explore node is active, then the module generate a smal reward for one shot hiebbian learning. If the exploit node is active, then the module only propagate the value of goal without reward and act as a retrieval mechanism for the current goal. More etails in the paper.

# Params

```
Topic field : Ros topic to where to send the goal
Topic reward : Ros topic to where to send the reward signal
Topic retrieve : Ros topic to where to send the retrieve signal
```