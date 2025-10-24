REM connects dobot magician to WSL2
usbipd bind --hardware-id 10c4:ea60 
usbipd attach --wsl Ubuntu-22.04 --hardware-id 10c4:ea60

REM Opens a WSL2 terminal
wsl -e gnome-terminal