
all:
	hlint .
	stack build

run:
	stack run