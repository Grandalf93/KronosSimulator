## Add New costum world
In order to load a new scenario, it is recommended to follow the nexts steps:
1. Create a `.world` file. This should be placed in the  `worlds` folder of the ROS package
```sh
cd /workspace/rover/ros2/src/simulator/kiwi_gazebo/worlds/
/workspace/rover/ros2/src/simulator/kiwi_gazebo/worlds$ touch 3_LMU.world
```
2. Add the spherical_coordinates to georeference the new world
```xml
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>33.9717</latitude_deg>
      <longitude_deg>-118.415</longitude_deg>
      <elevation>0</elevation>
      <heading_deg>180</heading_deg>
    </spherical_coordinates>
```
3. Since the repository works with satellite images. It is necessary to load the plugin `StaticMap` by adding the following lines as sugested by [gazebo](https://classic.gazebosim.org/tutorials?tut=static_map_plugin&cat=build_world)
```xml
     <plugin name="map" filename="libStaticMapPlugin.so">
      <center>37.386491 -122.065255</center>
      <world_size>100</world_size>
      <map_type>satellite</map_type>
      <api_key>enter_your_google_api_key_here</api_key>
      <use_cache>false</use_cache>
    </plugin>
```
> Note: remember to center the world with the latitude and longitude of the center of the map/scenario desired. Please contact the AI team in case you want to add a new world.

You can also add the next template to your world file: 

```xml
<sdf version='1.7'>
    <world name='default'>
        
        <spherical_coordinates>
            <surface_model>EARTH_WGS84</surface_model>
            <world_frame_orientation>ENU</world_frame_orientation>
            <latitude_deg>33.9717</latitude_deg>
            <longitude_deg>-118.415</longitude_deg>
            <elevation>0</elevation>
            <heading_deg>180</heading_deg>
        </spherical_coordinates>  
       
        <light name='sun' type='directional'>
          <pose>0 0 1000 0 -0 0</pose>
          <cast_shadows>1</cast_shadows>
          <diffuse>0.8 0.8 0.8 1</diffuse>
          <specular>0.5 0.5 0.5 1</specular>
          <attenuation>
            <range>1000</range>
            <constant>0.9</constant>
            <linear>0.01</linear>
            <quadratic>0.001</quadratic>
          </attenuation>
          <direction>-0.5 0.1 -0.4</direction>
          <spot>
            <inner_angle>0</inner_angle>
            <outer_angle>0</outer_angle>
            <falloff>0</falloff>
          </spot>
        </light>

        <plugin name="map" filename="libStaticMapPlugin.so">
            <center>37.386491 -122.065255</center>
            <world_size>100</world_size>
            <map_type>satellite</map_type>
            <api_key>enter_your_google_api_key_here</api_key>
            <use_cache>false</use_cache>
        </plugin>
        
        <include>
            <pose>-18 3 0.33 0 0 0</pose>
            <uri>model://map_satellite_37.386491_-122.065255_100</uri>
        </include>         
        
        <gravity>0 0 -9.8</gravity>
        <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
        <atmosphere type='adiabatic'/>
        <physics type='ode'>
          <max_step_size>0.001</max_step_size>
          <real_time_factor>1</real_time_factor>
          <real_time_update_rate>1000</real_time_update_rate>
        </physics>
        <scene>
          <ambient>0.4 0.4 0.4 1</ambient>
          <background>0.7 0.7 0.7 1</background>
          <shadows>1</shadows>
        </scene>
        <wind/>
        
        
        <!------insert the models here ----->

    
    </world>    
</sdf>
```
> **NOTE:** **MAKE SURE YOU DON'T POST SECRET API KEYS TO THE REPO**

## Interfacing the world 

### initial setup
1. Create a [github account](https://github.com/signup?ref_cta=Sign+up&ref_loc=header+logged+out&ref_page=%2F&source=header-home) 
2. Set the ssh keys so that you can clone the repository. [Configure SSH keys](https://www.youtube.com/watch?v=s6KTbytdNgs)
3. Once you have configured your account, please request permits to clone the repository. Contact the AI team
4. Create a folder where to clone the repositories in. Open the terminal and use the command `mkdir`
![CreateDir](https://user-images.githubusercontent.com/104376530/178031659-f1c9b659-8b74-441b-9cb8-856245e4cd80.gif)


5. If you already have permissions. Open the folder in the terminal and clone the repository with the command `git clone git@github.com:kiwicampus/KronosSimulator.git.` 

```sh
cd <folder_with_repositories>
git clone git@github.com:kiwicampus/KronosSimulator.git
```

![CloningRepo](https://user-images.githubusercontent.com/104376530/178033910-eec2e56a-62dd-4875-9807-130830304acd.gif)

6. Open the folder in Visual Studio Code. It will build the container automacally if installed all the dependencies correctly. The building process will take quite a while so please be patient.

7. *If the devcontainer didn't run or the package was created from scratch*. you'll see a window on the bottom right corner, click in "reopen in container" button, if you don't see anything press `Ctrl+Shift+P` and type `Remote-Containers: Rebuild and Reopen in container` or `Docker-images: Build Image` option. When the container is opened, and executed for the first time or when there are changes on it, you can go for a walk because the building image process will start and it'll take a while due to the installation of all packages and dependencies of the dev-environment as [ROS2](https://index.ros.org/doc/ros2/), [Python](https://www.python.org/), and more stuff related to, while the process is complete here are some videos of [puppies](https://www.youtube.com/watch?v=mRf3-JkwqfU). You can see at any time the logs of the building process by clicking in `Starting with Dev Container` in the bottom right corner. When the process is done you'll see some messages about process succeeding.
 
<img src="https://user-images.githubusercontent.com/43115782/87437367-d5806200-c5b3-11ea-9bf2-836e45f46ed8.gif" alt="building_dev-container" width="1200">

8. Please create a new branch with the name of the campus you want to map i.e: `git checkout -b <the_campus_name>`

[comment]: <![Branch](https://user-images.githubusercontent.com/104376530/178044398-d828eaaf-6fd9-40f6-b29a-ce78b29e6b2e.gif)>

9. Set the user and email of your user with the following terminal commands

```sh
git config --global user.name "FIRST_NAME LAST_NAME"
git config --global user.email "MY_NAME@example.com"
```

10. Run the script in the visual studio terminal using the command

```sh!
bash start.sh
```
![Start](https://user-images.githubusercontent.com/104376530/178040887-f9c3457f-310c-4866-bfc3-b117a2545b99.gif)


### World camera
To move around the world click on and hold and drag the pointer to translate the camera view. In the event of camera rotation, press and hold the `shift` key as displacing the mouse.

> **NOTE:** Do not rotate the camera view with scroll.

### Add a sidewalk model
As for rendering a sidewalk model, it is necessary to follow the next steps:

1. Open the Interface window and click on `Sidewalk` button. Please make sure `Sidewalk`is on the screen as the model
2. Open Gazebo and locate the sidewalk to be rendered
3. Locate the vertex of the sidewalk.
4. Scroll-click on the vertex of the sidewalk
5. Right-click to spawn the created model


![SpawningSidewalk](https://user-images.githubusercontent.com/104376530/177643908-e9847ef4-a96a-4549-893f-4428eb89a85e.gif)

### Add a grass model
As for rendering a grass model, it is necessary to follow the next steps:

1. Open the Interface window and click on `Grass` button. Please make sure `Grass`is on the screen as the model
2. Open Gazebo and locate the grass to be rendered
3. Locate the vertex of the grass.
4. Scroll-click on the vertex of the sidewalk
5. Right-click to spawn the created model

![SpawningGrass](https://user-images.githubusercontent.com/104376530/177652279-310ff672-9190-4b6b-aefc-e33d2c0f4e55.gif)


### Add a wall model
As for rendering a wall model, it is necessary to follow the next steps:

1. Open the Interface window and click on `Wall` button. Please make sure `Wall`is on the screen as the model
2. Open Gazebo and locate the wall to be rendered
3. Locate the vertex of the wall.
4. Scroll-click on the vertex of the wall
5. Right-click to spawn the created model 


![SpawningWall](https://user-images.githubusercontent.com/104376530/177655350-92cea2ad-ac04-46b6-8411-dfd4c4b85706.gif)

### Add a building model
As for rendering a sidewalk model, it is necessary to follow the next steps:

1. Open the Interface window and click on `Building` button. Please make sure `Building`is on the screen as the model
2. Open Gazebo and locate the building to be rendered
3. Locate the vertex of the building.
4. Scroll-click on the vertex of the building
5. Right-click to spawn the created model

![SpawningBuilding](https://user-images.githubusercontent.com/104376530/177656409-59f13be3-dae1-4877-90a5-b3c761c9f209.gif)


### Add a tree model
As for rendering a tree model, it is necessary to follow the next steps:

1. Open the Interface window and click on `Tree` button. Please make sure `Tree`is on the screen as the model
2. Open Gazebo and locate the position of thre
4. Scroll-click on the point of the building
5. Right-click to spawn the created model 

> **NOTE:** Just a single scroll-click is necessary. In case you have selected more points, the tree will be spawned in the first scroll click.

![SpawningTree](https://user-images.githubusercontent.com/104376530/177657672-b3c0d8b3-bfc2-4241-893b-19954b598550.gif)

### Inserting models 
When adding a model, the xml format of the model will be populated from top to buttom in the document `SavedData.sdf` which is generated in the root of the workspace. Please open the file 'SavedData.sdf' and select all the content by pressing `ctrl + a`. Ident the document with `Shift` key and copy the items by pressing `ctrl + c`or right-click and clic on `Copy`. Once the content has been copied, open the world file you have created l and paste the content with `ctrl + v` below the `<wind/>` tag and in between the  `</world>` & `</world>` tags 

![ModelInsertion](https://user-images.githubusercontent.com/104376530/177884526-009e92bb-b278-4b9d-9924-26a7c219ef9a.gif)


Please, do not forget to save the file of the world before adding new elements. You can do so by pressing `ctrl + s` or click on file->save

### Commiting and pushing changes


Please  eventually  push all the changes made during the session to the remote repository. That is, save the changes to the cloud. Click on `source control` located on the left bar menu of VSC. Stage all changes to be added on the cloud and generate a message (commit) indicating, in broad terms, what the update is about. Then press `sync changes`


![CommitingChanges](https://user-images.githubusercontent.com/104376530/178043792-d7080d18-a010-4e5d-9769-e69b7f50d958.gif)

> **NOTE:** If there is a change  you don't want to stage, click on `Discard file`



### Troubleshooting 
In the event a non-desired model is spawned, please select/click the model to be deleted on Gazebo. Remove the model by either pressing `supr` key or right-click and click on `Delete`. Then open the file `SavedData.sdf` and delete the xml element. Remember the file is populated from top to buttom so the first xml element will correspond to the last model inserted. If you need to remove previous models, click on the model and check its name on the upper left menu of Gazebo in the `World` section. The name will be displayed in orange color. Look for the name of the model in the file `SavedData.sdf` by pressing `ctrl + f` and typing down the name.

`SavedData.sdf` 
![RemovingModel](https://user-images.githubusercontent.com/104376530/177885094-37aea1e2-9283-4611-97f4-6e0d522d9ae7.gif)

In the case you have forgotten to save `SavedData.sdf`  when removing the model(s). The file  will run into a conflict and won't update new inserted models. Press `ctrl + s` in the file and compare both versions. Select the right version and press `Use ypur changes and overwrite the content` if you want to update with newer versions

![Recording 2022-07-08 at 10 21 21](https://user-images.githubusercontent.com/104376530/178022457-d17619e9-ec09-4ffa-ba6d-41a823edc4d6.gif)





