cd
cd MOS-Brain/scripts
gnome-terminal -x bash -c "python server.py"
gnome-terminal -x bash -c "python 1.py"
gnome-terminal -x bash -c "python 2.py"
gnome-terminal -x bash -c "python 3.py"
gnome-terminal -x bash -c "python 4.py"
gnome-terminal -x bash -c "python 5.py"
gnome-terminal -x bash -c "python 6.py"
gnome-terminal -x bash -c "cd ..;python -m http.server 8080"