# Hardware Design Files

This folder contains all the hardware design files for the **LANDER** module (Open Robot Battery), organized for easy access and reproduction.

## 📁 Folder Structure

```
hardware/
├── schematics/          # Electrical schematics and circuit diagrams
├── gerber/             # PCB manufacturing files
├── bom/                # Bill of Materials (component lists)
├── cad/                # 3D mechanical models and CAD files
└── images/             # Project images and diagrams
```

## 📋 File Types

### Schematics (`schematics/`)
- **LANDER-schematics.pdf** - Complete electrical circuit diagrams for the LANDER module
- **Source files** - Original schematic files (KiCad, Altium, etc.)

### Manufacturing Files (`gerber/`)
- **LANDER-gerber-files.zip** - PCB manufacturing files for LANDER module fabrication
- **Drill files** - Hole drilling specifications
- **Panel files** - Multi-board panels for automated assembly

### Bill of Materials (`bom/`)
- **LANDER-BOM.xlsx** - Complete component list with part numbers and suppliers
- **LANDER-BOM.csv** - Machine-readable format for import into other systems

### CAD Files (`cad/`)
- **LANDER-CAD-files.zip** - Complete mechanical design package
- **3D Models** - Mechanical design files (STEP, STL)
- **Source CAD** - Original design files (SolidWorks, Fusion 360, etc.)

### Images (`images/`)
- **Project photos** - Assembly and usage images
- **Diagrams** - Technical illustrations and block diagrams

## 🛠 Manufacturing Notes

- **PCB Thickness**: 1.6mm
- **Copper Weight**: 1oz
- **Surface Finish**: HASL or ENIG recommended
- **Solder Mask**: Green (standard)
- **Silkscreen**: White

## 📦 Component Sourcing

All components in the BOM are selected from major electronics distributors for:
- Easy sourcing and availability
- Competitive pricing
- Quality assurance
- Lead time reliability

## ⚠️ Important Notes

- This is a **proof-of-concept** design for laboratory use only
- Not certified for commercial applications
- Follow all safety guidelines in the technical manual
- Use proper ESD precautions when handling components

## 🔗 Related Documentation

- [Technical Manual](../MANUAL.md) - Complete assembly and usage instructions
- [Main README](../README.md) - Project overview and getting started guide