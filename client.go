package zmodbus

import (
	"bufio"
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
		Address:  mc.Conf.URL,
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

func (mc *ModbusClient) ReadLogsFromMemory(SlaveId uint8, LogIndex int) (Values []uint16, err error) {

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

	Values = bytesToUint16s(BIG_ENDIAN, Response[3:ResponseLength-2])

	return Values, nil
}
