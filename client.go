package zmodbus

import (
	"bufio"
	"encoding/binary"
	"fmt"
	"net"
	"sync"
	"time"

	"github.com/goburrow/serial"
)

type ClientConfiguration struct {
	URL      string
	Speed    uint
	DataBits uint
	Parity   string
	StopBits uint
	Timeout  time.Duration
}

type ModbusClient struct {
	Conf   ClientConfiguration
	lock   sync.Mutex
	Stream serial.Port
}

func NewClient(cc *ClientConfiguration) (mc *ModbusClient, err error) {
	mc = &ModbusClient{
		Conf: *cc,
	}
	return
}

func (mc *ModbusClient) Open() (err error) {
	mc.lock.Lock()
	defer mc.lock.Unlock()

	mc.Stream, err = serial.Open(&serial.Config{
		Address:  fmt.Sprintf("\\\\.\\%s", mc.Conf.URL),
		BaudRate: int(mc.Conf.Speed),
		DataBits: int(mc.Conf.DataBits),
		Parity:   mc.Conf.Parity,
		StopBits: int(mc.Conf.StopBits),
		Timeout:  mc.Conf.Timeout,
	})
	return err
}

func (mc *ModbusClient) Close() (err error) {
	return mc.Stream.Close()
}

func (mc *ModbusClient) ReadHoldingRegisters(SlaveId uint8, addr uint8, Qty uint8) (Values []uint16, err error) {

	var crc crc
	crc.init()
	AddrHighLow := uint16ToBytes(BIG_ENDIAN, uint16(addr))
	QtyHighLow := uint16ToBytes(BIG_ENDIAN, uint16(Qty))
	ResponseLength := 3 + 2 + (2 * Qty)

	cmd := []byte{SlaveId, 0x3, AddrHighLow[0], AddrHighLow[1], QtyHighLow[0], QtyHighLow[1]}
	crc.add(cmd)
	cmd = append(cmd, crc.value()...)

	_, err = mc.Stream.Write(cmd)
	if err != nil {
		return
	}

	Reader := bufio.NewReader(mc.Stream)
	Buffer := make([]byte, 256)
	var Response []byte

	for {
		n, err := Reader.Read(Buffer)
		if err != nil {
			if netErr, ok := err.(net.Error); ok && netErr.Timeout() {
				return nil, err
			} else {
				return nil, err
			}
		}
		Response = append(Response, Buffer[:n]...)
		if len(Response) >= int(ResponseLength) {
			break
		}
	}

	if len(Response) < 5 {
		return nil, fmt.Errorf("invalid response length")
	}

	// Check for exception response
	if Response[1] == (0x03 + 0x80) { // 0x83 for "Read Holding Registers"
		exceptionCode := Response[2]
		return nil, fmt.Errorf("modbus exception %d received", exceptionCode)
	}

	// compute the CRC on the entire frame, excluding the CRC
	crc.init()
	crc.add(Response[0 : ResponseLength-2])

	// compare CRC values
	if !crc.isEqual(Response[ResponseLength-2], Response[ResponseLength-1]) {
		return nil, fmt.Errorf("Bad CRC")
	}

	Values = bytesToUint16s(BIG_ENDIAN, Response[3:ResponseLength-2])

	return Values, nil
}

func (mc *ModbusClient) ReadLogsFromMemory(SlaveId uint8, LogIndex int) (Values []int16, err error) {

	var crc crc
	crc.init()
	LogIndexHighLow := uint16ToBytes(BIG_ENDIAN, uint16(LogIndex))
	ResponseLength := 84

	cmd := []byte{SlaveId, 0x8, 0, 0, LogIndexHighLow[0], LogIndexHighLow[1]}
	crc.add(cmd)
	cmd = append(cmd, crc.value()...)

	_, err = mc.Stream.Write(cmd)
	if err != nil {
		return
	}

	Reader := bufio.NewReader(mc.Stream)
	Buffer := make([]byte, 256)
	var Response []byte

	for {
		n, err := Reader.Read(Buffer)
		if err != nil {
			if netErr, ok := err.(net.Error); ok && netErr.Timeout() {
				return nil, err
			} else {
				return nil, err
			}
		}
		Response = append(Response, Buffer[:n]...)
		if len(Response) >= int(ResponseLength) {
			break
		}
	}

	if len(Response) < 5 {
		return nil, fmt.Errorf("invalid response length")
	}

	// Check for exception response
	if Response[1] == (0x08 + 0x80) { // 0x83 for "Read Holding Registers"
		exceptionCode := Response[2]
		return nil, fmt.Errorf("modbus exception %d received", exceptionCode)
	}

	// compute the CRC on the entire frame, excluding the CRC
	crc.init()
	crc.add(Response[0 : ResponseLength-2])

	// compare CRC values
	if !crc.isEqual(Response[ResponseLength-2], Response[ResponseLength-1]) {
		return nil, fmt.Errorf("Bad CRC")
	}

	Response = Response[:len(Response)-2]

	var converted []int16
	for i := 4; i < len(Response); i = i + 2 {
		converted = append(converted, int16(bytesToUint16(BIG_ENDIAN, []byte{Response[i], Response[i+1]})))
	}

	return converted, nil
}

func (mc *ModbusClient) WriteHoldingRegisters(slaveID uint8, startAddr uint16, values []uint16) error {

	var crc crc
	crc.init()

	// Function code 16 (0x10) is used for writing to holding registers
	functionCode := uint8(0x10)

	// Calculate the number of bytes that hold the register values
	byteCount := len(values) * 2 // 2 bytes per register

	// Prepare the request
	request := make([]byte, 7+byteCount)
	request[0] = slaveID
	request[1] = functionCode
	binary.BigEndian.PutUint16(request[2:], startAddr)
	binary.BigEndian.PutUint16(request[4:], uint16(len(values))) // number of registers
	request[6] = uint8(byteCount)

	// Add the register values to the request
	for i, val := range values {
		binary.BigEndian.PutUint16(request[7+2*i:], val)
	}

	// Calculate and append CRC
	crc.add(request)
	request = append(request, crc.value()...)

	// Write request to the serial port
	_, err := mc.Stream.Write(request)
	if err != nil {
		return err
	}

	// Read and process the response
	reader := bufio.NewReader(mc.Stream)
	buffer := make([]byte, 256)
	var response []byte

	for {
		n, err := reader.Read(buffer)
		if err != nil {
			if netErr, ok := err.(net.Error); ok && netErr.Timeout() {
				return err
			}
			return err
		}
		response = append(response, buffer[:n]...)
		if len(response) >= 8 {
			break
		}
	}

	// Validate response
	if len(response) < 8 {
		return fmt.Errorf("invalid response length")
	}

	// Check for exception response
	if response[1] == (functionCode + 0x80) {
		exceptionCode := response[2]
		return fmt.Errorf("modbus exception %d received", exceptionCode)
	}

	// Verify response CRC
	crc.init()
	crc.add(response[:6]) // Response length minus CRC
	if !crc.isEqual(response[6], response[7]) {
		return fmt.Errorf("bad CRC in response")
	}

	// Additional checks can be performed here, such as verifying the echoed address and register quantity

	return nil
}
