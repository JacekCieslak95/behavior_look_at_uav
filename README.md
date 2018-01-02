# Behavior BehaviorLookAtUAV
Paczka tworzy behavior używany przez Aerostack (oprogramowanie grupy Vision4UAV: https://github.com/Vision4UAV/Aerostack)
Zachowanie umożliwiające utrzymanie obrotu (yaw) zadanego drona (opierając się na jego pozycji).
### Instalacja ###
1. Pliki niniejszego repozytorium należy umieścić w folderze 
    `~/workspace/ros/aerostack_catkin_ws/src/`
    tak, aby tworzyły poniższe drzewo:
    
        ~/workspace/ros/aerostack_catkin_ws/src/
            -behavior_look_at_uav
    		    -CMakeLists.txt
                -package.xml
                -launch
                    -behavior_look_at_uav.launch
    			-src
                    -include
                        -behavior_look_at_uav.h
                    -source
                        -behavior_look_at_uav.cpp
                        -behavior_look_at_uav.cpp

2. Przeprowadzić kompilację catkin `~/workspace/ros/aerostack_catkin_ws/$ catkin_make`
3. Edytować plik `simulated_quadrotor_basic.sh` - W skrypcie uruchamiającym należy dokleić na końcu poniższe linie:
    
	    `#----------------------------------------------` \
	    `# Behavior BehaviorLookAtUAV                                   ` \
	    `#----------------------------------------------` \
	    --tab --title "Behavior LookAtUAV" --command "bash -c \"
	    roslaunch behavior_look_at_uav behavior_look_at_uav.launch --wait \
    		drone_id_namespace:=drone$NUMID_DRONE \
    		drone_id:=$NUMID_DRONE \
    		my_stack_directory:=${AEROSTACK_STACK};
    	exec bash\"" \
    
4. Edytować plik `behavior_catalog.yaml`. Plik znajduje się w lokalizacji: `~/workspace/ros/aerostack_catkin_ws/src/aerostack_stack/configs/droneX` 
    W sekcji `behavior_descriptors` należy dokleić poniższe linie:
#### UWAGA! Należy to wkleić do folderu `configs/droneX` każdego drona, którego chcemy uruchamiać z danym zachowaniem.
#### Np. Używając tego w dronach 1 i 2 poniższy fragment należy dokleić do `behavior_catalog.yaml` w folderach `configs/drone1` oraz `configs/drone2`
	    
		
          - behavior: LOOK_AT_UAV
            incompatible_lists: [motion_behaviors, SETPOINT_BASED_FLIGHT_CONTROL]
            arguments:
              - argument: ANGLE
                allowed_values: [-360,360]
              - argument: DRONE_ID
                allowed_values: [0,10]
		    
				
##### UWAGA! Wcięcia powinny być realizowane przez spacje, nie tabulatory!
##### UWAGA! Zachowanie nie będzie działać, jeśli jednocześnie uruchomione jest inne zachowanie wykorzysyujące DroneTrajectoryController

### Przyjmowane argumenty ###
Behavior przyjmuje argumenty:
    
    droneID=x
    
Jest to numer drona, za którym dron będzie podążał
    
    angle=y
    
Jest to kąt wzgędem obrotu (yaw) danego drona