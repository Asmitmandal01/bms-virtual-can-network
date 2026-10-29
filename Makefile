.PHONY: all sim test clean

all: sim test

sim:
	python3 run_simulation.py

test:
	python3 -m unittest discover -s tests -v

clean:
	rm -rf __pycache__ src/__pycache__ tests/__pycache__
