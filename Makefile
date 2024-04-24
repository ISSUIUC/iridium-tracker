CC = gcc

SRC = Software/decoding/decoder.c

EXEC = a.out

all: $(EXEC)

$(EXEC): $(SRC)
	$(CC) $(SRC) -o $(EXEC)

run:
	$(CC) $(SRC) -o $(EXEC) && ./a.out

clean:
	rm -f $(EXEC)
