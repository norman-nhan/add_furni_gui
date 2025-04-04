# add_furni_gui
This package is a rviz panel plugin. After catkin build, open **Rviz** > choose **Panel** > choose **Add New Panel** > choose **AddFurniPanel**, and then **AddFurniPanel** will be added.

# Manual
## How to use the panel
There are 2 tabs: **File** and **Add Plane**
- **File** is for opening and saving file.
- **Add Plane** is for modifying furniture.

**Add Plane Tab**

- The name appears on **Furniture name** box is **Selected furniture**. This means that you are editing that furniture. The attribute you can change are as follow:
    - plane: coordinate of furniture.
    - belong: the room/area in which furniture belongs to.
    - access:
- Choose old furniture to edit by chosing the name from **Furniture name** box or write a new name to add new furniture.
- Write or choose where furniture belongs to in **Belong** box.
- To add/change plane attribute of a furniture. DO THE FOLLOWING:
  1. Hit the **Record Plane** button.
  2. Choose **Publish point** tool or press `c`.
  3. Click on map to add that point.
  4. Repeat step 2 and 3, 4-times.
- You can delete a furniture by clicking **Delete** button.

**Importances when choosing plane points**
- When adding/changing plane, the next point should be on the same side of furniture, don't go diagonally.
- The first 2 points will be assigned as access attribute.

# Importances related to add_furni_gui plugin workflow
This plugin needs to run a ros node because it can't read YAML file by standalone. 
This node will read YAML file and send it to **Rviz panel** via **OpenFurniFile** service.

**Run node by**
```
rosrun add_furni_gui add_furni_gui_node
```
or
```
roslaunch add_furni_gui test.launch
```
