/*
 * polling.c
 *
 * Code for polling USART1
 *  Created on: 05.01.2018
 *      Author: jonas
 */


/*
deadloopprev = 0;
while ((USART_GetFlagStatus(USART1, USART_FLAG_RXNE ) == RESET) &&
	 (deadloopprev < 1000))
{
  deadloopprev++;
}
deadloopprev = 0;
while ((USART_GetFlagStatus(USART1, USART_FLAG_RXNE ) == SET) &&
	 (deadloopprev < 1000))
{
	uint8_t NMEA_start = 0;
	uint8_t sectionCount = 0;
	for (int i = 0; (i<NMEA_stringlength-1)&&(sectionCount<7); i++ )
	{
		uint16_t data_received;
		if (USART_GetFlagStatus(USART1, USART_FLAG_RXNE ) == SET)
		{
			data_received = USART_ReceiveData(USART1);
			USART_ClearFlag(USART1, USART_FLAG_RXNE);
			if (data_received <= 0xFF)
			{
				switch (data_received)
				{
					case '$':
						if (i<2) i=0;
						NMEA_start = 1;
						sectionCount++;
						break;
					case '\r':
						NMEA_start = 0;
						break;
					case '\n':
						NMEA_start = 0;
						break;
					default:
						break;
				}
				if(NMEA_start)
					NMEA_string[i] = data_received;
				else i--;
			}
		}
		else if (NMEA_string[i] == 0 && i>0) i--;
	}
	deadloopprev++;
}
deadloopprev = 0;
*/
