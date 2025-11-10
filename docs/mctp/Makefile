.PHONY: test
test: tla2tools.jar openbmc_usb_mctp.tla openbmc_usb_mctp.cfg
	java -XX:+UseParallelGC -jar tla2tools.jar openbmc_usb_mctp.tla

tla2tools.jar:
	curl -OL https://github.com/tlaplus/tlaplus/releases/download/v1.7.4/tla2tools.jar
	echo bee4a54f3ee3d4afc347c3240ec2d9e93b075104 tla2tools.jar | sha1sum --check -
