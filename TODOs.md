# TODOs
## Primäres Ziel
- Single Robot Navigation implementieren und über RViz Navigation zu Zielposen ermöglichen

## Namespace-Konzept
- Namespaces werden in Scenario-Dateien definiert und an einzelne Pakete weitergegeben
- In top-level Launch Dateien der einzelnen Pakete wird PushNamespace für setzen des Namespaces für alle Child-Nodes verwendet

## Paket-spezifische Aufgaben

### auna_nav2
- Standard nav2 Launch-Dateien nutzen, außer bringup.launch.py
  - Custom bringup.launch.py mit Komponenten-Flags (Lokalisierung, SLAM, Nav)
- Launch-Datei Refactoring:
  - Konsolidierung auf multi_robot.launch.py
  - Single-Robot Launch als Basis für Multi-Robot Setup
- Migration von nav2 zu auna_nav2 sicherstellen (Sicherstellen, dass nicht noch irgendwo nav2 verwendet wird)

### auna_gazebo
- Launch-Struktur refaktorisieren:
  1. spawn_robot.launch.py (Basis)
  2. spawn_single_robot.launch.py (+ RSP & LocalizationPose)
  3. spawn_multi_robot.launch.py (n-fache Single-Robot Instanzen)
- Das Spawnen der Roboter soll nur über die spawn_multi_robot.launch.py erfolgen. Die anderen beiden Launch Dateien werden nur intern verwendet.

### auna_common
- Utility-Script für Namespace-Parameter validieren
- Navigation-Parameterdateien überprüfen
