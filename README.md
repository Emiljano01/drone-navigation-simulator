# 🚁 Autonomous Drone Navigation Simulator

Simulim 3D izometrik i navigimit autonom të një droni, i ndërtuar me Python dhe Pygame. Përdor algoritmin A* (A-star) për gjetjen e rrugës optimale midis pengesave, me sensorë virtualë dhe radar në kohë reale.

## ✨ Funksionalitete

- 🎯 Algoritmi **A\* Pathfinding** për navigim autonom optimal
- 🎨 Grafikë izometrike 3D me blloqe dinamike dhe efekte vizuale
- 📡 Sistem sensorësh (8 drejtime) për zbulimin e pengesave
- 🖥️ HUD interaktiv me radar, bateri, dhe log në kohë reale
- 🕹️ Ndërveprim i plotë: vendos pengesa, cakto destinacione, kontrollo dronin
- ⚡ Simulim fizik: shpejtësi, bateri, "bob" i realizmit të fluturimit

## 🎮 Kontrollet

| Veprim | Kontroll |
|--------|----------|
| Cakto destinacion | Klik i djathtë (mouse) |
| Shto/hiq pengesë | Klik i majtë (mouse) |
| Nis/ndalo fluturimin | `SPACE` |
| Reset i plotë | `R` |
| Gjenero pengesa të reja | `G` |
| Dil | `ESC` ose `Q` |

## 🛠️ Teknologjitë e përdorura

- **Python 3.13**
- **Pygame** – motori grafik dhe input handling
- **Algoritmi A\*** – pathfinding me heapq (priority queue)
- **Matematikë**: projeksion izometrik, trigonometri për animacione

## 🚀 Si të nisësh projektin lokalisht

1. Klono repository-n:
```bash
git clone https://github.com/Emiljano01/drone-navigation-simulator.git
cd drone-navigation-simulator
```

2. Krijo virtual environment dhe aktivizoje:
```bash
python -m venv .venv
.venv\Scripts\activate      # Windows
source .venv/bin/activate   # Mac/Linux
```

3. Instalo librarite:
```bash
pip install -r requirements.txt
```

4. Nise aplikacionin:
```bash
python main.py
```