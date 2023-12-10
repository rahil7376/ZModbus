package zmodbus

type serialPortConfig struct {
	Device   string
	Speed    uint
	DataBits uint
	Parity   uint
	StopBits uint
}
