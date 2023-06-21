# ParaNut: Config creator

Config creator is tool that allows the user to modify/create the configuration 
file of the ParaNut processor.

Additional features:
- Saving presets of the configuration
- Approximating resource usage 

## Prerequisites

To get things started we will try to run a very simple GTK based GUI 
application using the PyGObject provided Python bindings. First create a small 
Python script called hello.py with the following content and save it somewhere:

```python
import gi
gi.require_version("Gtk", "3.0")
from gi.repository import Gtk

window = Gtk.Window(title="Hello World")
window.show()
window.connect("destroy", Gtk.main_quit)
Gtk.main()
```

Before we can run the example application we need to install PyGObject, GTK and 
their dependencies. Follow the instructions for your platform below.

After running the example application have a look at the “Python GTK 3 Tutorial”
for more examples on how to create GTK apps and the “PyGObject API Reference” 
for API documentation for all supported libraries.

## Ubuntu/ Debian

Installing the system provided PyGObject:
1. Open a terminal
2. Execute 
```bash
sudo apt install python3-gi python3-gi-cairo gir1.2-gtk-3.0
```
3. Change the directory to where your `hello.py` script can be found (e.g. cd Desktop)
4. Run 
```bash
python3 hello.py
```

Installing from PyPl with pip:
1. Open a terminal and enter your virtual environment
2. Execute to install the build dependencies and GTK
```bash
sudo apt install libgirepository1.0-dev gcc libcairo2-dev pkg-config python3-dev gir1.2-gtk-3.0
```
3. Execute to build and install Pycairo
```bash
pip3 install pycairo
``` 
4. Execute to build and install PyGObject
```bash
pip3 install PyGObject
```
5. Change the working directory to where your `hello.py` script can be found
6. Run 
```bash
python3 hello.py
```

## Installing ParaNut: Config creator

`pn-config-creator` is included in the ParaNut installation. To install ParaNut 
run `make && make install` from the ParaNut root direcotry. 

There is also the option to install only the `pn-config-creator`. The 
installation process is described in the next section.

### Manual installation
[Skip](#using-paranut-config-creator) this steps if `pn-config-creator` was 
installed with the ParaNut installer. 

1. Execute to create directory.
```bash
sudo mkdir -p /opt/paranut/tools
```
2. Execute to change the ownership. 
```bash
sudo chowon $USER /opt/paranut
```
3. Execute to create directory.
```bash
mkdir -p /opt/paranut/tools
```
4. Execute 
```bash
cp -R /.../paranut/config-creator /opt/paranut
```
5. Execute 
```bash
cp -R /.../paranut/tools/bin /opt/paranut/tools
```
6. Execute 
```bash
cp /.../paranut/settings.sh /opt/paranut
```
7. Sourse file 
```bash
source /opt/paranut/settings.sh
```

## Using ParaNut: Config creator

To use the tool source the setting file in the paranut root directory:
```bash
source /opt/paranut/settings.sh
```
To run the program execute"
```bash
pn-config-creator
```

## License
This project uses the following license: BSD 2-Clause License
