# Uncomment lines below if you have problems with $PATH
#
#SHELL := /bin/zsh
#PATH := /usr/local/bin:$(PATH)
all:
	pio -f -c vim run
upload:
	pio -f -c vim run --target upload
clean:
	pio -f -c vim run --target clean
uploadfs:
	pio -f -c vim run --target uploadfs
update:
	pio -f -c vim update
commit: 
	@echo git commit -am $(msg)
push:	
	git push --set-upstream origin dev-alpha

#your own cmd
#program:
#	pio -f -c vim run --target program



