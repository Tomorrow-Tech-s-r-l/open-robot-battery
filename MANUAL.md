# Open Robot Battery - Technical Manual

## Purpose of the Board

The purpose of this PCB is to provide a proof-of-concept (POC) charging solution for generic devices using a USB-C interface. The board is intended for laboratory and demonstration purposes, providing:

- Safe charging of low-power devices from a standardized USB-C port
- Flexibility to adapt to various input and output power requirements
- A compact design to fit within an existing powerbank slot

> **⚠️ Important**: This board is not certified for commercial use and is designed for internal testing, demonstration, and prototyping only.

## Main Function Blocks

| Block | Description | Key Components |
|-------|-------------|----------------|
| **USB-C Power Delivery Controller** | This is the core of the board. It manages USB-C negotiation and ensures correct voltage/current delivery to the connected device. | CYPD3177, associated resistors and capacitors, controllable by the multiposition switches |
| **Boost Module** | Converts and regulates the input voltage from the powerbank to 24V for the coil solenoid | Boost converter |
| **Powerbank Mechanical Docking** | Provides mechanical and electrical connection for the powerbank | Spring ejector, coil solenoid, USB C connector* |
| **I2C Interface** | To access status and control register | Pin strip connection |

> *Only for the POC, the future release will be different

## How It Works

### Power Input
- The board receives voltage from the connected powerbank
- The board is designed to be compatible with the Amperry powerbank that supports USB-C PD up to 20V

### USB-C Negotiation
- The CYPD3177 acts as the USB-C controller
- The controller negotiates the required voltage/current with the powerbank

### Voltage/Current Regulation and Output Delivery
There are 2 multiposition switches (SW1 and SW2) that allow the user to select some resistor combinations that set the output voltage and current limit.

> **⚠️ Important**: The output voltage (V) and current limit (I) must be set through the switch before connecting the load. The mechanical switch can temporarily disconnect the resistive load when moving from one position to another; during this brief interval, the output voltage automatically rises to the maximum available voltage (20V). Connecting a load before setting V/I can damage the device.

### Powerbank Ejection
- The board contains a boost converter that raises the supply to 24V for the solenoid
- When the push-button is pressed, the solenoid coil is energized, pulling a plunger
- The mechanical spring then ejects the powerbank from its slot
- To ensure that the coil solenoid stays active only for a short time enough to unlock the powerbank (even if the button is pressed for a long time) there is an ad hoc RC filter
- The solenoid is de-energized after the ejection cycle

## Assembly Instructions

This project package includes two main items:
- **Individual PCB** – the proof-of-concept board for multidevice recharge
- **PCB Panel** – a larger panel containing multiple copies of the board, separated by V-score lines, designed to facilitate automatic component placement during production

### Hardware Files Location

All hardware design files are organized in the `hardware/` folder:
- **Schematics**: `hardware/schematics/LANDER-schematics.pdf`
- **Gerber Files**: `hardware/gerber/LANDER-gerber-files.zip`
- **Bill of Materials**: `hardware/bom/LANDER-BOM.xlsx`
- **CAD Files**: `hardware/cad/LANDER-CAD-files.zip`

### Component Assembly
- All components were selected from major electronics distributors for easy sourcing (see BOM in `hardware/bom/`)
- **Manual soldering**: Most components, including resistors, capacitors, connectors, and the boost converter, can be soldered by hand without specialized equipment
- **CYPD3177**: This IC requires careful soldering due to its fine pitch; automated reflow or precision soldering is recommended

### Assembly Steps
1. Detach the individual PCB from the V-scored panel along the designated lines
2. Populate components starting with the lowest profile parts (resistors, capacitors), followed by connectors and the boost converter
3. Place the CYPD3177 last, using appropriate soldering techniques to ensure proper alignment and connection
4. Inspect all solder joints for bridges or cold soldering, paying special attention to the USB-C controller and high-current paths

## Testing

1. Before connecting a load, set the output voltage and current using SW1 and SW2
2. Apply power from a compatible USB-C power source
3. Verify correct voltage at the output terminals with a multimeter
4. Test the powerbank ejection mechanism without a load first to ensure the solenoid operates correctly
5. Once verified, the board is ready for controlled laboratory use with target devices

## Safety Warnings

> **⚠️ Important**: This board is a proof-of-concept prototype intended for controlled laboratory use only. It is not certified for commercial or consumer applications.

### Electrical Safety

- **Voltage spikes**: When changing the output voltage/current with the switches, the board may briefly output the maximum available voltage (~20 V). Always set the desired V/I before connecting a load
- **Risk of damage**: Connecting devices before setting the correct voltage/current may damage the load or the board
- **High-current paths**: The solenoid and boost converter can draw significant current. Avoid touching exposed conductors during operation
- **Short circuits**: Do not short the output terminals. This can damage the board and connected power sources

### Mechanical Safety

- The board contains a spring-eject mechanism and solenoid. Do not place fingers or objects near the moving parts during testing to prevent pinching or injury
- Ensure the board is securely mounted on a stable surface during testing

### Component Handling

- The CYPD3177 and other ICs are sensitive to ESD. Use proper anti-static precautions (wrist strap, ESD mat) when handling
- Soldering should be performed in a well-ventilated area. Avoid inhaling solder fumes

### Usage Limitations

- The board is designed for low-power devices only. Do not connect high-power or commercial devices
- **Environmental conditions**: Operate only in dry, controlled laboratory environments. Avoid exposure to water, high humidity, or uncontrolled loads
- No CE, EMC, or other certifications are included. Users are responsible for ensuring safe lab practices