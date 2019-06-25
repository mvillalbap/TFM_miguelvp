package main

import (
	"flag"
	"fmt"
	"os"
	"os/signal"
	"sync"
	"time"
	"bufio"
	"strconv"
	"strings"

	"github.com/kidoman/embd"
	_ "github.com/kidoman/embd/host/rpi"

	"github.com/carloop/simulator-program/mcp2515"
)

type CarData struct {
    speed   int64
    engine  float64
    temp    int64
    vin     string

    pcb		bool
}

func main() {
    dataIn := make(chan string, 1)
    car := new(CarData)
    car.pcb = false
	flag.Parse()

	err := embd.InitSPI()
	if err != nil {
		panic(err)
	}
	defer embd.CloseSPI()

	const (
		device  = 0
		speed   = 1e5
		bpw     = 8
		delay   = 0
		channel = 0
	)

	spi := embd.NewSPIBus(embd.SPIMode0, device, int(speed), bpw, delay)
	defer spi.Close()

	canDevice := mcp2515.New(spi)
	err = canDevice.Setup(500000)

	if err != nil {
		printError(err)
	} else {
		car.pcb = true
	}

    paint(car, true)

	rxChan := make(mcp2515.MsgChan, 10)
	txChan := make(mcp2515.MsgChan, 10)
	errChan := make(mcp2515.ErrChan, 10)
	respondChan := make(mcp2515.MsgChan, 10)
	startChan := make(chan bool, 1)

	var wg sync.WaitGroup

	if car.pcb {
		wg.Add(1)
		go func() {
			defer wg.Done()
			mcp2515.RunMessageLoop(canDevice, rxChan, txChan, errChan)
		}()
		wg.Add(1)
		go func() {
			defer wg.Done()
			printCanMessages(rxChan, txChan, errChan, respondChan)
		}()
		wg.Add(1)
		go func() {
			defer wg.Done()
			sendMessages(respondChan, txChan, startChan, car)
		}()
	} else {
		wg.Add(1)
		go func() {
			defer wg.Done()
			test(rxChan, txChan, car)
		}()
	}

	wg.Add(1)
	go func() {
		defer wg.Done()
		read(dataIn)
	}()

	wg.Add(1)
	go func() {
		defer wg.Done()
		update(dataIn, car, startChan)
	}()

	// Wait for all goroutines to be done
	wg.Wait()
}

func printCanMessages(rxChan mcp2515.MsgChan, txChan mcp2515.MsgChan,
	errChan mcp2515.ErrChan, respondChan mcp2515.MsgChan) {

	fmt.Println("Starting CAN receiver")

	startTime := time.Now()

	c := make(chan os.Signal, 1)
	signal.Notify(c, os.Interrupt)

	for {
		select {
		case rxMessage := <-rxChan:
			printMessage(rxMessage, startTime)
			respondChan <- rxMessage
		case err := <-errChan:
			printError(err)
		case <-c:
			// Program done
			return
		}
	}
}

func printMessage(message *mcp2515.Message, startTime time.Time) {
	timeOffset := message.Time.Sub(startTime).Seconds()
	fmt.Printf("%15.6f %03x %d", timeOffset, message.Id, message.Length)
	for i := uint8(0); i < message.Length; i++ {
		fmt.Printf(" %02x", message.Data[i])
	}
	fmt.Println("")

}

func printError(err error) {
	fmt.Printf("Error occured: %v", err)
	fmt.Println("")
}

func sendMessages(rxChan mcp2515.MsgChan, txChan mcp2515.MsgChan, startChan chan bool, coche *CarData) {
	
	//startTime := time.Now()
	
	c := make(chan os.Signal, 1)
	signal.Notify(c, os.Interrupt)

	start := false

	//i := uint8(0)
	for {
		/*var message mcp2515.Message
		/*message.Id = 0x2AA
		/*message.Length = 8
		for j := 0; j < 8; j++ {
			message.Data[j] = 0xAA
		}
		i += 1*/

		select {
		case start = <-startChan:
			fmt.Printf("Ignition: %t\n", start)
			fmt.Printf("")

		case message := <-rxChan:
			//printMessage(message, startTime)
			//fmt.Printf("Mensaje recibido con ignition: %t\n", start)
			// If the car has been started
			if (start) {
				// Message added to queue
				//fmt.Println("Contestamos")
				respond(txChan, message, coche)
			}

		case <-c:
			// Program done
			return

		default:
			// If tx channel is full, ignore
		}

		time.Sleep(10 * time.Millisecond)

	}
}

func paint(coche *CarData, clear bool) {
    if clear {
        fmt.Printf("\u001b[1;1H\u001b[2J")
        if !coche.pcb {
        	fmt.Println("*** No MCP2515 detected ***\n")
        }
        fmt.Printf("\n\tSpeed (Km/h): %d\t\t\tEngine RPM: %2.2f\t\t\n", coche.speed, coche.engine)
        fmt.Printf("\n\n\tAir Ambient Temperature (ºC): %d\t\tVIN: %s\t\t\n\n", coche.temp, coche.vin)
    }
}

func read(dataIn chan string) {
	c := make(chan os.Signal, 1)
	signal.Notify(c, os.Interrupt)

	for {
		select {
		case <-c:
			return

		default:
			reader := bufio.NewReader(os.Stdin)
	        text, _ := reader.ReadString('\n')
	        dataIn <- text

		}
    }
}

func update(dataIn chan string, car *CarData, startChan chan bool) {
	c := make(chan os.Signal, 1)
	signal.Notify(c, os.Interrupt)

    for {
	    i := "                      "
	    select {
	    case <-c:
	    	return

        case i = <-dataIn :
            if len(i) > 6 {
                if i[:6] == "engine" {
                    car.engine, _ = strconv.ParseFloat(i[7:len(i)-1], 64)
                    paint(car, true)
                } else if i[:5] == "speed" {
                    car.speed, _ = strconv.ParseInt(i[6:len(i)-1], 10, 64)
                    paint(car, true)
                } else if i[:3] == "air" {
                    car.temp, _ = strconv.ParseInt(i[4:len(i)-1], 10, 64)
                    paint(car, true)
                } else if i[:3] == "vin" {
                	if i[4:10] == "preset" {
                		if i[11:12] == "1" {
                			car.vin = "VF1BG0A0524085422"
                		} else if i[11:12] == "2" {
                			car.vin = "3FADP4FJ2BM113913"
                		}
                	} else {
                    	car.vin = strings.ToUpper(i[4:len(i)-1])
                	}
                    paint(car, true)
                } else if i[:5] == "start" {
                	startChan <- true
                } else if i[:4] == "stop" {
                	startChan <- false
                }
            } else if len(i) > 5 {
                if i[:5] == "speed" {
                    car.speed, _ = strconv.ParseInt(i[6:len(i)-1], 10, 64)
                    paint(car, true)
                } else if i[:3] == "air" {
                    car.temp, _ = strconv.ParseInt(i[4:len(i)-1], 10, 64)
                    paint(car, true)
                } else if i[:3] == "vin" {
                    car.vin = strings.ToUpper(i[4:len(i)-1])
                    paint(car, true)
                } else if i[:5] == "start" {
                	startChan <- true
                } else if i[:4] == "stop" {
                	startChan <- false
                }
            } else if len(i) > 3 {
                if i[:3] == "air" {
                    car.temp, _ = strconv.ParseInt(i[4:len(i)-1], 10, 64)
                    paint(car, true)
                } else if i[:3] == "vin" {
                    car.vin = strings.ToUpper(i[4:len(i)-1])
                    paint(car, true)
                } else if i[:5] == "start" {
                	startChan <- true
                } else if i[:4] == "stop" {
                	startChan <- false
                }
            } else {
                fmt.Printf("Invalid command\r\nHelp: \nspeed xxxx -> Sets a speed\nengine xxxx -> Sets a rpm\nair xxxx -> Sets an air temperature\nvin xxxx -> Sets a VIN\nstart\n\n")
                fmt.Printf("Some VIN examples preset are:\nvin preset 1 -> Renault Laguna II 2002: VF1BG0A0524085422\nvin preset 2 -> Ford Fiesta 2011: 3FADP4FJ2BM113913\n")
            }
	    }
	}
}

func respond(txChan mcp2515.MsgChan, mensaje *mcp2515.Message, coche *CarData) {
	var actualBmens, actualBresp uint8
	var i, j, length uint8

	length = mensaje.Data[0]
	actualBresp = 2
	
	if mensaje.Data[0] == 0x30 {
		for j = 1; j <= 2; j++ {
			var respuesta mcp2515.Message
			respuesta.Id = 0x7e8
			respuesta.Length = 8
			respuesta.Data[0] = 0x20 + j
			for i = 1; i < 8; i++ {
				respuesta.Data[i] = uint8([]rune(coche.vin)[(j-1)*7+2+i])
			}

			respuesta.Time = time.Now()

			printMessage(&respuesta, mensaje.Time)
			txChan <- &respuesta
		}
	} else {
		var respuesta mcp2515.Message
		respuesta.Id = 0x7e8
		respuesta.Length = 8
		respuesta.Data[1] = mensaje.Data[1] + 0x40
		
		for actualBmens = 2; actualBmens <= length; actualBmens++ {
			// PID service 01
			if mensaje.Data[1] == 0x01 {

				respuesta.Data[actualBresp] = mensaje.Data[actualBmens]
				actualBresp++;
				// Speed
				if mensaje.Data[actualBmens] == 0x0d {
					respuesta.Data[actualBresp] = uint8(coche.speed)
					actualBresp++;

				// RPM
				} else if (mensaje.Data[actualBmens] == 0x0c) {
					respuesta.Data[actualBresp] = uint8(uint64(coche.engine *4) >> 8)
					actualBresp++;
					respuesta.Data[actualBresp] = uint8(uint64(coche.engine *4))
					actualBresp++;

				// Ambient Air
				} else if mensaje.Data[actualBmens] == 0x46 {
					respuesta.Data[actualBresp] = uint8(coche.temp + 40)
					actualBresp++;

				// PIDs 1
				} else if mensaje.Data[actualBmens] == 0x00 {
					respuesta.Data[actualBresp] = 0x00
					respuesta.Data[actualBresp+1] = 0x18
					respuesta.Data[actualBresp+2] = 0x00
					respuesta.Data[actualBresp+3] = 0x00
					actualBresp += 4
				
				// PIDs 2
				} else if mensaje.Data[actualBmens] == 0x20 {
					respuesta.Data[actualBresp] = 0x00
					respuesta.Data[actualBresp+1] = 0x00
					respuesta.Data[actualBresp+2] = 0x00
					respuesta.Data[actualBresp+3] = 0x00
					actualBresp += 4

				// PIDs 3
				} else if mensaje.Data[actualBmens] == 0x40 {
					respuesta.Data[actualBresp] = 0x00
					respuesta.Data[actualBresp+1] = 0x00
					respuesta.Data[actualBresp+2] = 0x40
					respuesta.Data[actualBresp+3] = 0x00
					actualBresp += 4

				// PIDs 4
				} else if mensaje.Data[actualBmens] == 0x60 {
					respuesta.Data[actualBresp] = 0x00
					respuesta.Data[actualBresp+1] = 0x00
					respuesta.Data[actualBresp+2] = 0x00
					respuesta.Data[actualBresp+3] = 0x00
					actualBresp += 4

				// PIDs 5
				} else if mensaje.Data[actualBmens] == 0x80 {
					respuesta.Data[actualBresp] = 0x00
					respuesta.Data[actualBresp+1] = 0x00
					respuesta.Data[actualBresp+2] = 0x00
					respuesta.Data[actualBresp+3] = 0x00
					actualBresp += 4

				// PIDs 6
				} else if mensaje.Data[actualBmens] == 0xA0 {
					respuesta.Data[actualBresp] = 0x00
					respuesta.Data[actualBresp+1] = 0x00
					respuesta.Data[actualBresp+2] = 0x00
					respuesta.Data[actualBresp+3] = 0x00
					actualBresp += 4
				}
				respuesta.Data[0] = actualBresp - 1


			} else if mensaje.Data[1] == 0x09 {

				// VIN data extended message format
				if mensaje.Data[actualBmens] == 0x02 {
					respuesta.Data[0] = 0x10
					respuesta.Data[1] = 0x14
					respuesta.Data[actualBresp] = mensaje.Data[1] + 0x40
					actualBresp++;
					respuesta.Data[actualBresp] = mensaje.Data[actualBmens]
					actualBresp++;
					respuesta.Data[actualBresp] = 0x01
					actualBresp++;
					for i = actualBresp; i < 8; i++ {
						respuesta.Data[i] = uint8([]rune(coche.vin)[i - actualBresp])
					}

				// PIDs 1
				} else if mensaje.Data[actualBmens] == 0x00 {
					actualBmens++;
					respuesta.Data[actualBresp] = mensaje.Data[1] + 0x40
					actualBresp++;
					respuesta.Data[actualBresp] = mensaje.Data[actualBmens]
					actualBresp++;
					respuesta.Data[actualBresp] = 0x40
					respuesta.Data[actualBresp+1] = 0x00
					respuesta.Data[actualBresp+2] = 0x00
					respuesta.Data[actualBresp+3] = 0x00
					actualBresp += 4
					respuesta.Data[0] = actualBresp - 1
				}

			}
		}

		respuesta.Time = time.Now()

		printMessage(&respuesta, mensaje.Time)
		txChan <- &respuesta
	}

}

func test(rxChan mcp2515.MsgChan, txChan mcp2515.MsgChan, coche *CarData) {
	c := make(chan os.Signal, 1)
	signal.Notify(c, os.Interrupt)

	i := uint8(0)
	for {
		var message mcp2515.Message
		message.Id = 0x7df
		message.Length = 2
		
		message.Data[0] = 0x01
		message.Data[1] = 0x0d
		message.Data[2] = 0x00

		i += 1

		select {
		case txChan <- &message:
			// Message added to queue

		case <-c:
			// Program done
			return
		default:
			// If tx channel is full, ignore
		}

		time.Sleep(10 * time.Millisecond)

	}
}