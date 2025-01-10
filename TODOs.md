# TODOs
## Ziel:
- Single Robot Navigation ans laufen bekommen und über rviz navigation an Zielpose starten können

## Namespaces:
- über die Scenario Dateien soll ein namespace festgelegt werden. Dieser Namespace wird and die launch Dateien der einzelnen Packete weitergegeben, welche dann PushNamespace verwenden um den Namespace an alle Child-Nodes weiterzugeben

## Packete:
### auna_nav2
- Verwendung der launch Dateien aus dem standard nav2 package, bis auf der bringup.launch.py
    - Idee ist, dass die custom bringup.launch.py über Flags genauer steuern kann, welche Komponenten gestartet werden sollen (z.B. nur Lokalisierung ohne SLAM und Nav)
- Refactoring der single und multi robot launch Dateien
    - Es wird nur noch die multi robot launch Datei verwendet, welche dann die single robot launch Datei n Mal startet
- Sicherstellen, dass das auna_nav2 package statt dem standard nav2 package verwendet wird
### auna_gazebo
- Launch Dateien für single und multi robot spawn sollen ähnlich zu auna_nav2 refaktorisiert werden
    - spawn_robot.launch.py lädt einen Roboter in die Simulation
    - spawn_single_robot.launch.py wrappt spawn_robot.launch.py und startet extra Nodes wie RobotStatePublisher und LocalizationPosePublisher
    - spawn_multi_robot.launch.py wrappt spawn_single_robot.launch.py und ruft diese mehrfach auf um n Roboter zu spawnen
- auch hier soll der Namespace in der obersten launch Datei (spawn_multi_robot.launch.py) festgelegt werden und an alle Child-Nodes weitergegeben werden. Der Namespace wird über das Scenario festgelegt
### auna_common
- Prüfen, ob Utility Script zu setzen des Namespace in den Parameterfiles fuer die navigation noch funktioniert
