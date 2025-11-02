sudo apt-get install dconf-cli
sudo apt-get remove gnome-terminal
dconf reset -t /org/gnome/terminal
sudo apt-get install --reinstall bash-completion
sudo apt-get install gnome-terminal