# Simulating Security Attacks on Embedded Systems

## Inhalt des Repositories
Derzeit befindet sich nur ein Catkin-Workspace im Repository

---
## Mit dem Repository arbeiten

### Catkin-Workspace starten

1. Ladet euch das gesamte Repository herunter
    * entweder indem ihr "git clone https://github.com/Julian9619/security-attacks-fp.git" im Ordner (im Folgenden my_folder) eurer Wahl ausführt, oder
    * indem ihr den ZIP-Ordner downloaded und diesen im Ordner eurer Wahl entpackt
2. in my_folder: "cd security-attacks-fp/security_attacks_fp_ws"
3. "catkin_make"
4. nun ist euer Wokspace einsatzbereit und entspricht dem Muster aus dem Repository
5. In die Konsole "source ./devel/setup.bash"
6. Um Simulationen zu starten: "roslaunch launch/my_launch_file_name.launch"
    * z.B.: "roslaunch launch/collision_world.launch"
  
