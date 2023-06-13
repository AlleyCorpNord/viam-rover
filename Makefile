# for PI local install

all:
	sudo apt install virtualenv
	sudo apt install libportaudio2

	virtualenv venv
	./venv/bin/pip install -r requirements.txt