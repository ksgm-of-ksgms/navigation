
all:
	stack build

lint:
	hlint .

run:
	stack run