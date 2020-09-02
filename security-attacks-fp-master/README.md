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
  
### Dateien und Simulationen zum Repository hinzufügen

*Ändert nicht den main-Branch!!! Wollt ihr Änderungen vornehmen tut dies in einem neuen Branch. Vielleicht macht es Sinn wenn jeder für sich einen Branch anlegt und nur diesen bearbeitet und wir alle regelmäßig diese überprüfen und mittels pull-request den main-branch aktualisieren*

#### Mit Drag and Drop

1. vom Startbildschirm des Repositories auf github ausgehend ( **nicht im main-branch**):
2. öffnet den Ordner "security_attacks_fp_ws/**src**"
3. drückt "add file" und wählt anschließend "Upload files"
4. zieht den gesamten Ordner mit eurer Simulation in die dafür vorgesehene Fläche (dieser Ordner sollte die Ordner "src", "worlds" und optional "launch" (diesen solltet ihr entfernen) enthalten, sowie eine "CMakeList.txt" und "package.xml")
5. Betätigt "Commit Changes"
6. Geht zurück zu "security_attacks_fp_ws"
7. öffnet den "launch"-Ordner und nutzt Schritt 3 um eure launch-Datei hinzuzufügen (ihr könnt diese Datei auch in eurem neu hochgeladenen Ordner lassen, jedoch wäre ein einheitlicher Aufbau im Repository praktisch)

#### Über die Konsole
...das darf dann jemand anderes hier hinschreiben...


