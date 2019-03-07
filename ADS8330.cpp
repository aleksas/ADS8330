#include "ADS8330.h"

ADS8330::ADS8330(uint8_t _SelectPin, uint8_t _ConvertPin, uint8_t _EOCPin)
{
	SelectPin = _SelectPin;
	ConvertPin = _ConvertPin;
	EOCPin = _EOCPin;
	pinMode(ConvertPin, OUTPUT);
	digitalWrite(ConvertPin, HIGH);
	pinMode(SelectPin, OUTPUT);
	digitalWrite(SelectPin, HIGH);
	pinMode(EOCPin, INPUT);
	Vref = 2.5;
	EOCTimeout = 100000;
	ConnectionSettings = SPISettings(10000000, MSBFIRST, SPI_MODE0);
}

void ADS8330::setCommandBuffer(CommandRegister Command)
{
	CommandBuffer = 0;
	CommandBuffer = static_cast<uint8_t>( Command ) << 12;
}

void ADS8330::begin()
{
	setCommandBuffer(CommandRegister::WriteConfig);
	setConfiguration(ConfigRegisterMap::ChannelSelectMode, false);
	setConfiguration(ConfigRegisterMap::ClockSource, true);
	setConfiguration(ConfigRegisterMap::TriggerMode, true);
	setConfiguration(ConfigRegisterMap::EOCINTPolarity, true);
	setConfiguration(ConfigRegisterMap::EOCINTMode, true);
	setConfiguration(ConfigRegisterMap::ChainMode, true);
	setConfiguration(ConfigRegisterMap::AutoNap, true);
	setConfiguration(ConfigRegisterMap::Nap, true);
	setConfiguration(ConfigRegisterMap::Sleep, true);
	setConfiguration(ConfigRegisterMap::TAG, true);
	setConfiguration(ConfigRegisterMap::Reset, true);
	bitWrite(CommandBuffer, 8, true);
	//Serial.println(CommandBuffer,BIN);
	sendCommandBuffer(true);
	//sendWriteCommandBuffer();
}

void ADS8330::reset()
{
	setCommandBuffer(CommandRegister::WriteConfig);
	setConfiguration(ConfigRegisterMap::ChannelSelectMode, false);
	setConfiguration(ConfigRegisterMap::ClockSource, true);
	setConfiguration(ConfigRegisterMap::TriggerMode, true);
	setConfiguration(ConfigRegisterMap::EOCINTPolarity, true);
	setConfiguration(ConfigRegisterMap::EOCINTMode, true);
	setConfiguration(ConfigRegisterMap::ChainMode, true);
	setConfiguration(ConfigRegisterMap::AutoNap, true);
	setConfiguration(ConfigRegisterMap::Nap, true);
	setConfiguration(ConfigRegisterMap::Sleep, true);
	setConfiguration(ConfigRegisterMap::TAG, true);
	setConfiguration(ConfigRegisterMap::Reset, false);
	bitWrite(CommandBuffer, 8, true);
	//Serial.println(CommandBuffer,BIN);
	sendCommandBuffer(true);
	//sendWriteCommandBuffer();
}

void ADS8330::setConfiguration(ConfigRegisterMap Option, bool Setting)
{
	bitWrite(CommandBuffer, static_cast<uint8_t>(Option), Setting);
}

void ADS8330::setVref(float NewVref)
{
	Vref = NewVref;
}

float ADS8330::getVref()
{
	return Vref;
}

uint16_t ADS8330::getConfig()
{
	setCommandBuffer(CommandRegister::ReadConfig);
	return sendCommandBuffer(true);
	//return sendReadCommandBuffer();
}

void ADS8330::print_binary(uint32_t v)
{
	int mask = 0;
	int n = 0;
	int num_places = 32;
	for (n=1; n<=num_places; n++)
	{
		mask = (mask << 1) | 0x0001;
	}
	v = v & mask;  // truncate v to specified number of places
	while(num_places)
	{
		if (v & (0x0001 << (num_places-1) ))
		{
			Serial.print("1");
		}
		else
		{
			Serial.print("0");
		}
		--num_places;
	}
}

uint16_t ADS8330::sendCommandBuffer(bool SendLong)
{
	union DataConverter
	{
		uint16_t UIntLargeData;
		uint8_t UIntSmallData[2];
	};
	DataConverter TempInput;
	DataConverter TempOutput;
	TempOutput.UIntLargeData = CommandBuffer;
	SPI.beginTransaction(ConnectionSettings);
	SPI.transfer( 0 );
	digitalWrite(SelectPin,LOW);
	if (SendLong)
	{
		TempInput.UIntSmallData[1] = SPI.transfer( TempOutput.UIntSmallData[1] );
		TempInput.UIntSmallData[0] = SPI.transfer( TempOutput.UIntSmallData[0] );
	}
	else
	{
		TempInput.UIntSmallData[1] = SPI.transfer( TempOutput.UIntSmallData[1] );
	}
	digitalWrite(SelectPin, HIGH);
	SPI.endTransaction();
	return TempInput.UIntLargeData;
}

uint8_t ADS8330::getSample(float* WriteVariable, bool _UseChannel0)
{
	uint16_t IntegerValue = 0;
	uint8_t status = getSample(&IntegerValue, _UseChannel0);
	*WriteVariable = Vref * ( (float)(IntegerValue) / 65535.0);
	return status;
}

uint8_t ADS8330::getSample(uint16_t* WriteVariable, bool _UseChannel0)
{
	UseChannel0 = _UseChannel0;
	setSampleChannel();
	return getSampleInteger(WriteVariable);
}

void ADS8330::setSampleChannel()
{
	if (UseChannel0)
	{
		setCommandBuffer(CommandRegister::SelectCh0);
	}
	else
	{
		setCommandBuffer(CommandRegister::SelectCh1);
	}
	sendCommandBuffer(false);
}

uint8_t ADS8330::getSampleInteger(uint16_t* WriteVariable)
{
	if (!beginsent)
	{
		begin();
		beginsent = true;
	}
	union DataConverter
	{
		uint16_t UIntLargeData;
		uint8_t UIntSmallData[2];
	};
	DataConverter TempInput;
	DataConverter TempOutput;
	setCommandBuffer(CommandRegister::ReadData);
	TempOutput.UIntLargeData = CommandBuffer;
	uint32_t starttime = micros();
	bool keepwaiting = true;
	digitalWrite(ConvertPin, LOW);
	while(keepwaiting)
	{
		if (digitalRead(EOCPin) == 0)
		{
			keepwaiting = false;
		}
		else
		{
			if ( (micros() - starttime) > EOCTimeout)
			{
				digitalWrite(ConvertPin, HIGH);
				return 1;
			}
		}
	}
	digitalWrite(ConvertPin, HIGH);
	keepwaiting = true;
	SPI.beginTransaction(ConnectionSettings);
	while(keepwaiting)
	{
		if (digitalRead(EOCPin) == 1)
		{
			keepwaiting = false;
		}
		else
		{
			if ( (micros() - starttime) > EOCTimeout)
			{
				return 2;
			}
		}
	}
	digitalWrite(SelectPin,LOW);
	TempInput.UIntSmallData[1] = SPI.transfer( TempOutput.UIntSmallData[1] );
	TempInput.UIntSmallData[0] = SPI.transfer( TempOutput.UIntSmallData[0] );
	uint8_t TAGData = SPI.transfer( 0 );
	digitalWrite(SelectPin, HIGH);
	SPI.endTransaction();
	*WriteVariable = TempInput.UIntLargeData;
	if ( (uint8_t)(TAGData << 1) == (uint8_t)(0) && ( bitRead(TAGData,7) == UseChannel0 ) )
	{
		return 0;
	}
	else
	{
		return 3;
	}
}

SPISettings* ADS8330::GetSPISettings()
{
	return &ConnectionSettings;
}

/*


void ADS8330::sendShortCommandBuffer()
{
	union DataConverter
	{
		uint32_t UInt32Data;
		uint16_t UInt16Data[2];
		uint8_t UInt8Data[4];
	};
	DataConverter DataOutput;
	DataOutput.UInt16Data[1] = CommandBuffer;
	bitBangData(DataOutput.UInt32Data,ShortCommandLength);
}

void ADS8330::sendWriteCommandBuffer()
{
	union DataConverter
	{
		uint32_t UInt32Data;
		uint16_t UInt16Data[2];
		uint8_t UInt8Data[4];
	};
	DataConverter DataOutput;
	DataOutput.UInt16Data[1] = CommandBuffer;
	bitBangData(DataOutput.UInt32Data,WriteCommandLength);
}

uint16_t ADS8330::sendReadCommandBuffer()
{
	union DataConverter
	{
		uint32_t UInt32Data;
		uint16_t UInt16Data[2];
		uint8_t UInt8Data[4];
	};
	DataConverter DataOutput;
	DataConverter DataReturn;
	DataOutput.UInt16Data[1] = CommandBuffer;
	DataReturn.UInt32Data = bitBangData(DataOutput.UInt32Data,WriteCommandLength);
	return DataReturn.UInt16Data[0];
}

uint32_t ADS8330::bitBangData(uint32_t _send, uint8_t bitcount)
{
	pinMode(MISOPin, INPUT);
	pinMode(SelectPin, OUTPUT);
	pinMode(SCKPin, OUTPUT);
	pinMode(MOSIPin, OUTPUT);
	digitalWrite(SCKPin, LOW);
	uint32_t _receive = 0;
	uint8_t limit = 31-bitcount;
	Serial.print("O:");
	print_binary(_send);
	Serial.print("\n");
	for(int8_t i = 31; i>limit; i--)
	{
		digitalWrite(SelectPin, LOW);
		digitalWrite(MOSIPin, bitRead(_send, i));
		digitalWrite(SCKPin, HIGH);
		bitWrite(_receive, i, digitalRead(MISOPin));
		digitalWrite(SCKPin, LOW);
		if (bitRead(_send, i))
		{
			Serial.print("1");
		}
		else
		{
			Serial.print("0");
		}
	}
	//Serial.print("\n");
	_receive = _receive;
	digitalWrite(SelectPin, HIGH);
	Serial.print("I:");
	print_binary(_receive);
	Serial.print("\n");
	return _receive;
}

void ADS8330::getSamples(float* Channel0, float* Channel1)
{
	uint16_t Channel0Int;
	uint16_t Channel1Int;
	getSamples(&Channel0Int, &Channel1Int);
	*Channel0 = Vref * ( (float)(Channel0Int) / 65535.0);
	*Channel1 = Vref * ( (float)(Channel1Int) / 65535.0);
}


uint16_t ADS8330::sendCommandBuffer()
{
	union DataConverter
	{
		uint16_t UIntLargeData;
		uint8_t UIntSmallData[2];
	};
	DataConverter TempInput;
	DataConverter TempOutput;
	TempOutput.UIntLargeData = CommandBuffer;
	SPI.beginTransaction(ConnectionSettings);
	SPI.transfer( 0 );
	digitalWrite(SelectPin,LOW);
	TempInput.UIntSmallData[1] = SPI.transfer( TempOutput.UIntSmallData[1] );
	TempInput.UIntSmallData[0] = SPI.transfer( TempOutput.UIntSmallData[0] );
	SPI.endTransaction();
	digitalWrite(SelectPin, HIGH);
	return TempInput.UIntLargeData;
}

void ADS8330::getSamples(uint16_t* Channel0, uint16_t* Channel1)
{
	union DataConverter
	{
		uint16_t UIntLargeData;
		uint8_t UIntSmallData[2];
	};
	bool HaveChannel0Data = false;
	bool HaveChannel1Data = false;
	uint8_t AttemptCount = 0;
	uint8_t ResetCount = 0;
	DataConverter TempInput;
	DataConverter TempOutput;
	setCommandBuffer(CommandRegister::ReadData);
	TempOutput.UIntLargeData = CommandBuffer;
	while ( !( HaveChannel0Data && HaveChannel1Data ) )
	{
		SPI.beginTransaction(ConnectionSettings);
		SPI.transfer( 0 );
		digitalWrite(SelectPin,LOW);
		TempInput.UIntSmallData[1] = SPI.transfer( TempOutput.UIntSmallData[1] );
		TempInput.UIntSmallData[0] = SPI.transfer( TempOutput.UIntSmallData[0] );
		uint8_t TAGData = SPI.transfer( 0 );
		uint8_t BlankData = SPI.transfer( 0 );
		print_binary(TempInput.UIntSmallData[1]);
		Serial.print(":");
		print_binary(TempInput.UIntSmallData[0]);
		Serial.print(":");
		print_binary(TAGData);
		Serial.print(":");
		print_binary(BlankData);
		Serial.print("=");
		if ( ( (uint8_t)(TAGData << 1) == (uint8_t)(0) ) && ( (uint8_t)(BlankData)==(uint8_t)(0) ) )
		{
			if (bitRead(TAGData,7))
			{
				*Channel1 = TempInput.UIntLargeData;
				HaveChannel1Data = true;
				Serial.print("A");
			}
			else
			{
				*Channel0 = TempInput.UIntLargeData;
				HaveChannel0Data = true;
				Serial.print("B");
			}
		}
		else
		{
			Serial.print("N");
		}
		Serial.print("\n");
		digitalWrite(SelectPin, HIGH);
		AttemptCount++;
		if (AttemptCount > 16)
		{
			Serial.print("Failed to collect data for ADS8330.");
			Serial.print("\n");
			digitalWrite(SelectPin, HIGH);
			SPI.endTransaction();
			setCommandBuffer(CommandRegister::WakeUp);
			sendCommandBuffer();
			reset();
			begin();
			SPI.beginTransaction(ConnectionSettings);
			SPI.transfer( 0 );
			digitalWrite(SelectPin, LOW);
			ResetCount++;
			if (ResetCount > 4)
			{
				HaveChannel1Data = true;
				HaveChannel0Data = true;
				Serial.print("Failed to reboot ADS8330.");
				Serial.print("\n");
			}
		}
		SPI.endTransaction();
	}
}
*/

/*
uint16_t ADS8330::getSampleInteger(bool UseChannel0)
{
	if (UseChannel0)
	{
		setCommandBuffer(CommandRegister::SelectCh0);
	}
	else
	{
		setCommandBuffer(CommandRegister::SelectCh1);
	}
	sendCommandBuffer();
	setCommandBuffer(CommandRegister::ReadData);
	return sendCommandBuffer();
}
void ADS8330::begin()
{
	setCommandBuffer(CommandRegister::WriteConfig);
	setConfiguration(ConfigRegisterMap::ChannelSelectMode, false);
	setConfiguration(ConfigRegisterMap::ClockSource, true);
	setConfiguration(ConfigRegisterMap::TriggerMode, false);
	setConfiguration(ConfigRegisterMap::EOCINTPolarity, true);
	setConfiguration(ConfigRegisterMap::EOCINTMode, true);
	setConfiguration(ConfigRegisterMap::ChainMode, true);
	setConfiguration(ConfigRegisterMap::AutoNap, true);
	setConfiguration(ConfigRegisterMap::Nap, true);
	setConfiguration(ConfigRegisterMap::Sleep, true);
	setConfiguration(ConfigRegisterMap::TAG, false);
	setConfiguration(ConfigRegisterMap::Reset, true);
	Serial.println(CommandBuffer,BIN);
	sendCommandBuffer();
}
void ADS8330::getSample(float* WriteVariable, bool UseChannel0)
{
	uint16_t IntegerValue = getSampleInteger(UseChannel0);
	*WriteVariable = Vref * ( (float)(IntegerValue) / 65535.0);
}
void ADS8330::getSample(uint16_t* WriteVariable, bool UseChannel0)
{
	*WriteVariable = getSampleInteger(UseChannel0);
}
*/