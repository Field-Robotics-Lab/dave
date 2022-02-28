---
layout: default
title: Dave ROV Models
parent: Underwater Vehicle Models
grand_parent: Dave Models
nav_order: 2
---


# Examples of Vehicles

Collection of how to spawn existing models.

## Dave ROVs

### Smilodon

```
roslaunch smilodon_gazebo smilodon.launch
roslaunch uuv_dave joy_thrusterop.launch namespace:=smilodon
```

![smilodon](../images/smilodon1.png)

![smilodon_rear](../images/smilodon_rear.png)


### Caracara

```
roslaunch caracara_gazebo caracara.launch
roslaunch uuv_dave joy_thrusterop.launch namespace:=caracara
```

![caracara](../images/caracara.png)

![caracara2](../images/caracara2.png)



### Caldus

```
roslaunch caldus_gazebo caldus.launch
roslaunch uuv_dave joy_thrusterop.launch namespace:=caldus
```

![caldus](../images/caldus.png)


