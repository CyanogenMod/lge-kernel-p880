#ifndef	_LG_WIFI_CC_Table_
#define	_LG_WIFI_CC_Table_
#define _LG_WIFI_MAX_MAPPED_CCODE 8

typedef struct _wifi_cc_table {
	char ccode[4];
	char ccode_mapped[8];
	/* unsigned int mcc; */
} _wifi_cc_table_t;

_wifi_cc_table_t _WIFI_CC_TABLE[] = 
{
	{"AD", "GB" /*, 213  */  }, //Andorra
	{"AE", "KR/24" /*, 424  */  }, //UAE
	{"AF", "AF" /*, 412  */  }, //Afghanistan
	{"AG", "CA/2" /*, 344  */  }, //Antigua & Barbuda
	{"AI", "CA/2" /*, 365  */  }, //Anguilla
	{"AL", "GB" /*, 276  */  }, //Albania
	{"AM", "AO" /*, 283  */  }, //Armenia
	{"AN", "BR" /*, 362  */  }, //Netherlands Antilles
	{"AO", "AO" /*, 631  */  }, //Angola
	{"AR", "BR" /*, 722  */  }, //Argentina
	{"AS", "CA/2" /*, 544  */  }, //American Samoa (USA)
	{"AT", "GB" /*, 232  */  }, //Austria
	{"AU", "AU/2" /*, 505  */  }, //Australia
	{"AW", "KR/24" /*, 363  */  }, //Aruba
	{"AZ", "BR" /*, 400  */  }, //Azerbaijan
	{"BA", "GB" /*, 218  */  }, //Bosnia and Herzegovina
	{"BB", "BS" /*, 342  */  }, //Barbados
	{"BD", "BF" /*, 470  */  }, //Bangladesh
	{"BE", "GB" /*, 206  */  }, //Belgium
	{"BF", "BF" /*, 613  */  }, //Burkina Faso
	{"BG", "GB" /*, 284  */  }, //Bulgaria
	{"BH", "BS" /*, 426  */  }, //Bahrain
	{"BI", "AO" /*, 642  */  }, //Burundi
	{"BJ", "AO" /*, 616  */  }, //Benin
	{"BM", "CA/2" /*, 350  */  }, //Bermuda
	{"BN", "BS" /*, 528  */  }, //Brunei
	{"BO", "AO" /*, 736  */  }, //Bolivia
	{"BR", "BR" /*, 724  */  }, //Brazil
	{"BS", "BS" /*, 364  */  }, //Bahamas
	{"BT", "AO" /*, 402  */  }, //Bhntan
	{"BW", "GB" /*, 652  */  }, //Botswana
	{"BY", "GB" /*, 257  */  }, //Belarus
	{"BZ", "AO" /*, 702  */  }, //Belize
	{"CA", "CA/2" /*, 302  */  }, //Canada
	{"CD", "AO" /*, 630  */  }, //Congo. Democratic Republic of the
	{"CF", "AO" /*, 623  */  }, //Central African Republic
	{"CG", "AO" /*, 629  */  }, //Congo. Republic of the
	{"CH", "GB" /*, 228  */  }, //Switzerland
	{"CI", "AO" /*,   */  }, //Cote d'lvoire
	{"CK", "BR" /*, 548  */  }, //Cook Island
	{"CL", "BS" /*, 730  */  }, //Chile
	{"CM", "AO" /*, 624  */  }, //Cameroon
	{"CN", "BF" /*, 460  */  }, //China
	{"CO", "BR" /*, 732  */  }, //Columbia
	{"CR", "BR" /*, 712  */  }, //Costa Rica
	{"CU", "BR" /*, 368  */  }, //Cuba
	{"CV", "GB" /*, 625  */  }, //Cape Verde
	{"CX", "AU/2" /*,   */  }, //Christmas Island (Australia)
	{"CY", "GB" /*, 280  */  }, //Cyprus
	{"CZ", "GB" /*, 230  */  }, //Czech
	{"DE", "GB" /*, 262  */  }, //Germany
	{"DJ", "AO" /*, 638  */  }, //Djibouti
	{"DK", "GB" /*, 238  */  }, //Denmark
	{"DM", "BR" /*, 366  */  }, //Dominica
	{"DO", "BR" /*, 370  */  }, //Dominican Republic
	{"DZ", "KW/1" /*, 603  */  }, //Algeria
	{"EC", "BR" /*, 740  */  }, //Ecuador
	{"EE", "GB" /*, 248  */  }, //Estonia
	{"EG", "BS" /*, 602  */  }, //Egypt
	{"ER", "AO" /*, 657  */  }, //Eritrea
	{"ES", "GB" /*, 214  */  }, //Spain
	{"ET", "GB" /*, 636  */  }, //Ethiopia
	{"FI", "GB" /*, 244  */  }, //Finland
	{"FJ", "AO" /*, 542  */  }, //Fiji
	{"FM", "CA/2" /*, 550  */  }, //Federated States of Micronesia
	{"FO", "GB" /*, 288  */  }, //Faroe Island
	{"FR", "GB" /*, 208  */  }, //France
	{"GA", "AO" /*, 628  */  }, //Gabon
	{"GB", "GB" /*, 234  */  }, //United Kingdom
	{"GD", "BR" /*, 352  */  }, //Grenada
	{"GE", "GB" /*, 282  */  }, //Georgia
	{"GF", "GB" /*,   */  }, //French Guiana
	{"GH", "BR" /*, 620  */  }, //Ghana
	{"GI", "GB" /*, 266  */  }, //Gibraltar
	{"GM", "AO" /*, 607  */  }, //Gambia
	{"GN", "AO" /*, 611  */  }, //Guinea
	{"GP", "GB" /*, 340  */  }, //Guadeloupe
	{"GQ", "AO" /*, 627  */  }, //Equatorial Guinea
	{"GR", "GB" /*, 202  */  }, //Greece
	{"GT", "BS" /*, 704  */  }, //Guatemala
	{"GU", "CA/2" /*, 310  */  }, //Guam
	{"GW", "AO" /*, 632  */  }, //Guinea-Bissau
	{"GY", "QA" /*, 738  */  }, //Guyana
	{"HK", "BR" /*, 454  */  }, //Hong Kong
	{"HN", "BF" /*, 708  */  }, //Honduras
	{"HR", "GB" /*, 219  */  }, //Croatia
	{"HT", "BS" /*, 372  */  }, //Haiti
	{"HU", "GB" /*, 216  */  }, //Hungary
	{"ID", "QA" /*, 510  */  }, //Indonesia
	{"IE", "GB" /*, 272  */  }, //Ireland
	{"IL", "AO" /*, 425  */  }, //Israel
	{"IM", "GB" /*, 234  */  }, //Isle of Man
	{"IN", "BS" /*, 404  */  }, //India
	{"IQ", "AO" /*, 418  */  }, //Iraq
	{"IR", "AO" /*, 432  */  }, //Iran
	{"IS", "GB" /*, 274  */  }, //Iceland
	{"IT", "GB" /*, 222  */  }, //Italy
	{"JE", "GB" /*, 234  */  }, //Jersey
	{"JM", "GB" /*, 338  */  }, //Jameica
	{"JO", "XY/3" /*, 416  */  }, //Jordan
	{"JP", "JP/5" /*, 440  */  }, //Japan
	{"KE", "GB" /*, 639  */  }, //Kenya
	{"KG", "AO" /*, 437  */  }, //Kyrgyzstan
	{"KH", "BR" /*, 456  */  }, //Cambodia
	{"KI", "AU/2" /*, 545  */  }, //Kiribati
	{"KM", "AO" /*, 654  */  }, //Comoros
	{"KP", "AO" /*, 467  */  }, //North Korea
	{"KR", "KR/24" /*, 450  */  }, //South Korea
	{"KW", "KW/1" /*, 419  */  }, //Kuwait
	{"KY", "CA/2" /*, 346  */  }, //Cayman Islands
	{"KZ", "BR" /*, 401  */  }, //Kazakhstan
	{"LA", "KR/24" /*, 457  */  }, //Laos
	{"LB", "BR" /*, 415  */  }, //Lebanon
	{"LC", "BR" /*, 358  */  }, //Saint Lucia
	{"LI", "GB" /*, 295  */  }, //Liechtenstein
	{"LK", "BR" /*, 413  */  }, //Sri Lanka
	{"LR", "BR" /*, 618  */  }, //Liberia
	{"LS", "GB" /*, 651  */  }, //Lesotho
	{"LT", "GB" /*, 246  */  }, //Lithuania
	{"LU", "GB" /*, 270  */  }, //Luxemburg
	{"LV", "GB" /*, 247  */  }, //Latvia
	{"LY", "AO" /*, 606  */  }, //Libya
	{"MA", "KW/1" /*, 604  */  }, //Morocco
	{"MA", "GB" /*,   */  }, //Western Sahara (Morocco)
	{"MC", "GB" /*, 212  */  }, //Monaco
	{"MD", "GB" /*, 259  */  }, //Moldova
	{"ME", "GB" /*, 297  */  }, //Montenegro
	{"MF", "GB" /*,   */  }, //Saint Martin / Sint Marteen (Added on window's list)
	{"MG", "AO" /*, 646  */  }, //Madagascar
	{"MH", "BR" /*,   */  }, //Marshall Islands
	{"MK", "GB" /*, 294  */  }, //Macedonia
	{"ML", "AO" /*, 610  */  }, //Mali
	{"MM", "AO" /*,   */  }, //Burma (Myanmar)
	{"MN", "AO" /*, 428  */  }, //Mongolia
	{"MO", "BF" /*, 455  */  }, //Macau
	{"MP", "CA/2" /*,   */  }, //Northern Mariana Islands (Rota Island. Saipan and Tinian Island)
	{"MQ", "GB" /*, 340  */  }, //Martinique (France)
	{"MR", "GB" /*, 609  */  }, //Mauritania
	{"MS", "GB" /*,   */  }, //Montserrat (UK)
	{"MT", "GB" /*, 278  */  }, //Malta
	{"MU", "GB" /*, 617  */  }, //Mauritius
	{"MV", "BS" /*, 472  */  }, //Maldives
	{"MW", "BF" /*, 650  */  }, //Malawi
	{"MX", "BS" /*, 334  */  }, //Mexico
	{"MY", "BS" /*, 502  */  }, //Malaysia
	{"MZ", "BR" /*, 643  */  }, //Mozambique
	{"NA", "BR" /*, 649  */  }, //Namibia
	{"NC", "AO" /*, 546  */  }, //New Caledonia
	{"NE", "BR" /*, 614  */  }, //Niger
	{"NF", "BR" /*, 505  */  }, //Norfolk Island
	{"NG", "NG" /*, 621  */  }, //Nigeria
	{"NI", "BR" /*, 710  */  }, //Nicaragua
	{"NL", "GB" /*, 204  */  }, //Netherlands
	{"NO", "GB" /*, 242  */  }, //Norway
	{"NP", "SA" /*, 429  */  }, //Nepal
	{"NR", "AO" /*, 536  */  }, //Nauru
	{"NU", "BR" /*, 555  */  }, //Niue
	{"NZ", "BR" /*, 530  */  }, //New Zealand
	{"OM", "GB" /*, 422  */  }, //Oman
	{"PA", "BS" /*, 714  */  }, //Panama
	{"PE", "BR" /*, 716  */  }, //Peru
	{"PF", "GB" /*, 547  */  }, //French Polynesia (France)
	{"PG", "XY/3" /*, 537  */  }, //Papua New Guinea
	{"PH", "BR" /*, 515  */  }, //Philippines
	{"PK", "BF" /*, 410  */  }, //Pakistan
	{"PL", "GB" /*, 260  */  }, //Poland
	{"PM", "GB" /*, 308  */  }, //Saint Pierre and Miquelon
	{"PN", "GB" /*,   */  }, //Pitcairn Islands
	{"PR", "CA/2" /*, 330  */  }, //Puerto Rico (USA)
	{"PS", "BR" /*, 425  */  }, //Palestinian Authority
	{"PT", "GB" /*, 268  */  }, //Portugal
	{"PW", "BR" /*, 552  */  }, //Palau
	{"PY", "BR" /*, 744  */  }, //Paraguay
	{"QA", "BF" /*, 427  */  }, //Qatar
	{"RE", "GB" /*, 647  */  }, //Reunion (France)
	{"RKS", "AO" /*, 212  */  }, //Kosvo (Added on window's list)
	{"RO", "GB" /*, 226  */  }, //Romania
	{"RS", "GB" /*, 220  */  }, //Serbia
	{"RU", "BS" /*, 250  */  }, //Russia
	{"RW", "BF" /*, 635  */  }, //Rwanda
	{"SA", "SA" /*, 420  */  }, //Saudi Arabia
	{"SB", "AO" /*, 540  */  }, //Solomon Islands
	{"SC", "AO" /*, 633  */  }, //Seychelles
	{"SD", "GB" /*, 634  */  }, //Sudan
	{"SE", "GB" /*, 240  */  }, //Sweden
	{"SG", "BR" /*, 525  */  }, //Singapole
	{"SI", "GB" /*, 293  */  }, //Slovenia
	{"SK", "GB" /*, 231  */  }, //Slovakia
	{"SKN", "BF" /*, 356  */  }, //Saint Kitts and Nevis
	{"SL", "AO" /*, 619  */  }, //Sierra Leone
	{"SM", "GB" /*, 292  */  }, //San Marino
	{"SN", "GB" /*, 608  */  }, //Senegal
	{"SO", "AO" /*, 637  */  }, //Somalia
	{"SR", "AO" /*, 746  */  }, //Suriname
	{"SS", "GB" /*, 659  */  }, //South_Sudan
	{"ST", "AO" /*, 626  */  }, //Sao Tome and Principe
	{"SV", "BS" /*, 706  */  }, //El Salvador
	{"SY", "BR" /*, 417  */  }, //Syria
	{"SZ", "AO" /*, 653  */  }, //Swaziland
	{"TC", "GB" /*, 376  */  }, //Turks and Caicos Islands (UK)
	{"TD", "AO" /*, 622  */  }, //Chad
	{"TF", "GB" /*,   */  }, //French Southern and Antarctic Lands)
	{"TG", "AO" /*, 615  */  }, //Togo
	{"TH", "BR" /*, 520  */  }, //Thailand
	{"TJ", "AO" /*, 436  */  }, //Tajikistan
	{"TL", "BR" /*, 514  */  }, //East Timor
	{"TM", "AO" /*, 438  */  }, //Turkmenistan
	{"TN", "KW/1" /*, 605  */  }, //Tunisia
	{"TO", "AO" /*, 539  */  }, //Tonga
	{"TR", "GB" /*, 286  */  }, //Turkey
	{"TT", "BR" /*, 374  */  }, //Trinidad and Tobago
	{"TV", "AO" /*, 553  */  }, //Tuvalu
	{"TW", "TW/2" /*, 466  */  }, //Taiwan
	{"TZ", "BF" /*, 640  */  }, //Tanzania
	{"UA", "BS" /*, 255  */  }, //Ukraine
	{"UG", "BR" /*, 641  */  }, //Ugnada
	{"US", "CA/2" /*, 310  */  }, //US
	{"UY", "BR" /*, 748  */  }, //Uruguay
	{"UZ", "AO" /*, 434  */  }, //Uzbekistan
	{"VA", "GB" /*, 225  */  }, //Vatican (Holy See)
	{"VC", "BR" /*, 360  */  }, //Saint Vincent and the Grenadines
	{"VE", "BS" /*, 734  */  }, //Venezuela
	{"VG", "GB" /*, 348  */  }, //British Virgin Islands
	{"VI", "CA/2" /*,   */  }, //US Virgin Islands
	{"VN", "BR" /*, 452  */  }, //Vietnam
	{"VU", "AO" /*, 541  */  }, //Vanuatu
	{"WS", "SA" /*, 549  */  }, //Samoa
	{"YE", "AO" /*, 421  */  }, //Yemen
	{"YT", "GB" /*,   */  }, //Mayotte (France)
	{"ZA", "GB" /*, 655  */  }, //South Africa
	{"ZM", "BS" /*, 645  */  }, //Zambia
	{"ZW", "BR" /*, 648  */  }, //Zimbabwe
};

inline static char * _cpy_strcpy(char *d, const char *s)
{
	size_t i;

	for (i = 0; (d[i] = s[i]) != '\0'; i++);
	return d;
}

inline static int _cpy_strcmp(const char *s1, const char *s2)
{
    int i;
    int diff;

    for (i = 0;; i++)
    {
        diff = ((unsigned char *)s1)[i] - ((unsigned char *)s2)[i];
        if (diff != 0 || s1[i] == '\0')
            return diff;
    }
}

inline static int _getWiFiMappedCode( const char *ccode, char *ccode_mapped, int size )
{
	int bot = 0;
	int top = (int) ( sizeof( _WIFI_CC_TABLE ) / sizeof( _wifi_cc_table_t ));
	int mid = 0 ;
	
	// Check there are enough buffer.
	if( size < _LG_WIFI_MAX_MAPPED_CCODE )
	{
		return -2;
	}
	
	// Compair		
	while(bot <= top)
	{
		mid = (bot + top) / 2;       
		if ( _cpy_strcmp(_WIFI_CC_TABLE[mid].ccode, ccode) == 0 )
		{
			_cpy_strcpy( ccode_mapped, _WIFI_CC_TABLE[mid].ccode_mapped );
			return 0;
		}
		else if ( _cpy_strcmp(_WIFI_CC_TABLE[mid].ccode, ccode) > 0)
		{
			top = mid - 1;
		}
		else
		{
			bot = mid + 1;
		}  

	}
	
	// Can't find Country Code.
	return -1;
}
#endif