cd ~/MOS-Brain/visualization
gnome-terminal -x bash -c "python server.py"
gnome-terminal -x bash -c "python 1.py"
gnome-terminal -x bash -c "cd ..;python -m http.server 8080 --bind 0.0.0.0"