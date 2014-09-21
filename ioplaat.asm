 ;===============================================================================
; Universaalne IO-plaat modbus'i liinil. 8 digisisendit, 8 v�ljundit, 8 analoogsisendit
; Ftakt=11,0592 MHz.
; Side: 9600,n,8,2 modbus RTU
; 29.12.: lugemise, kirjutamise ja konfi k�sud testitud, OK. Koopia siit !
; analoogts�kli parendamine, WR reg0 samuti. Koopia!
; 1.1.2013.: justkui toimiks. Koodikaitse ka peale. Neemele testimisele. Koopia !
; 9.12.2012.: alustame...
; Register 0: binaarsed v�ljundid.
; MSB on DO mis on alati DO. Kirjutamisel muudab v�ljundi seisu (ei
; salvestata), lugemisel annab v�ljundi hetkeseisu. Kuidagi konfitav ei ole.
; LSB on UIO (ANA) osa. Kirjutamisel muudab vastava analoogkanali seisu
; kui ta on v�ljund (muidu ei teee midagi). Lugemisel annab samuti vastava
; analoogkanali seisu kui ta v�ljund. Kui DI siis annab samuti seisu
; arvestades debouncet ja sticky't. Kui pinn on analoogsisend, annab nulli.
;
; Register 1: binaarsed sisendid.
; MSB on DI mis on alati DI ja annab alati vastava sisendi seisu. Ainult
; loetav, ei ole konfitav. DI-l on debounce ja sticky reziim (k�igile
; korraga). Ei ole konfitav (s.t. debounce aeg)
; LSB on UIO (ANA) osa.Kui vastav analoogsisend on konfitud DI-na, siis
; annab selle seisu. Kui ei, siis vastav bitt=0. Kui konfitud v�ljundina,
; siis annab samuti hetke seisu ? Analoogpordi suund on konfitav registri
; 274 kaudu (1=v�ljund) bitthaaval. Pull-up'id konfitavad registri 273
; kaudu (DO seisu muudab ka!). M�lemad registrid salvestatakse EEPROMi ja
; loetakse stardil ning rakendatakse kohe. Kui UIO konfitud DI-na,
; kehtivad debounce ja sticky reziimid.
;
; Registrid 2..9. A/D muundi tulemused. Ainult loetavad. Ei ole konfitavad
; (otseselt). Kui m�ni kanal on DI v�i DO, siis annab nulli.
;
; Register adr 255. R-only, annab device type 0xF1
; Register adr 258. R-only, annab seerianumbri 1. osa (130 dec)
; Register adr 259. R-only, annab seerianumbri 2. osa (1..dec)
; Register adr 271. R/W-, annab analoog/digitaal omaduse. 1= digisisend, bitthaaval
; Register adr 272. R/W-, annab pull-up'ide seisu (ehk DO vaikimisi seisu), bitthaaval
; Register adr 273. R/W, seriali seaded nagu tabelis
; Register adr 274. R/W, seadme modbus aadress
; Register adr 275. R/W, ANA-pordi (UIO) suund, 1= v�ljund, bitthaaval)
;
; Serial
; -------
; b0,1,2 - kiirus, default 19200 (010)
; b3 -debounce k�igile sisenditele, default  ON
; b4 - sticky bitt k�igile sisenditele, default OFF
; b5,6 - ei kasuta
; b7 - paarsuse bitt, (0= even parity), default EVEN
;
; Peale 30-sekundist side puudumist v�i arusaamatuid k�ske l�heb vaikimisi
; seriali parameetritele (19200, even)
;------------------- LISA - RESETID ------------------
;reset1:
; - aeg s alates toite pealetulekust, mis peab olema �letatud. naiteks 600s
; - reset impulsi kestus s, n�iteks 1 s
; - ilma �hegi eduka modbus lugemiseta m��dunud aeg s, mille �letamisel (koos esimese tingimuse arvestamisega) impulss antakse. n�iteks 30 s.
;
;PWRSW:
; - aeg s alates toite tulekust, mis peab olema �letatud, n�iteks 10 s
; - reset impulsi kestus s. n�iteks 5 s
; - ilma �hegi eduka modbus lugemiseta m��dunud aeg s, mille �letamisel (koos esimese tingimuse arvestamisega) impulss antakse. n�iteks 10 s.
;------------------------ LISA - WIEGAND --------------------
;mul ei ole muude pikkustega kui 26 kokkupuudet olnud. aga barix on oma x8 moodulis niimoodi teinud
;et koodi pikkus on eraldi registris 11 (aadress j�relikult 10) kirjas. 
;wiegand kood registrites 12..16 (adr 11..15). saadud bitid nihutatakse registritesse paremalt 
;sisse alates reg12, viimase baidi vanemad bitid voivad valed/vanad olla, neid tuleb loetud 
; pikkuse abil maskida. jargmist wiegand koodi enne vastu ei voeta, kui reg 11 nullistatud, 
; 27.1.: resetid kodeeritud, peax kuidagi testima aga rauda eip ole mitte...
; 28.1.: Alustame Wiegandiga...Eelnevast tegin koopia.
;To check if WIEGAND data is available the register 11 has to be
;read. The lower Byte will contain the amount of Bits received.
;If it is 00h then no new data was received. The higher Byte
;contains a bitmap of found temperature sensors (see next page)
;The Bits of the received WIEGAND data are shifted in (from
;right to left) into the Registers 12 to 16 (5x16=80 bits max)
;--------- Wiegand --------------
;-------------------------- LISA - loendid igale DIN-ile -------------------------
; 3.2.: k�ik debouncega, sticky't ei ole. Aadressid alates 400-st. chk_len toimimise loogikat ei tea, lubab lugeda ja k�ik!
; 22.2.: read_setup'i parandamine, reg 666 access teeb n��d reseti.
; 27.2.: seriali kiiruse paranduse parandus, 3 v�ikest lisa. e_XOR peakx modb_write ja setupi puhul ka tulemuse enne porti kirjutamist xor-ima !
; 10.3.: register 767 teeb reseti, Enne oli 666 aga see j�i Dallase reg-te alla.
; 12.3.: Reg-te 271, 275 ja 0,1 toimimise t�psustamine. Koopia senisest.
; 28.3.: reset1 inversioonibitt PORTC,.3 lisatud.
; 6.4.: reload_side IGA baidi vastuv�tul ! Muudatused - CHG - V�idetalt tekib ajuti vastuse kadumine - uurida misp�rast.
; 6.4.: side viga fiksed (vt. main juurest). Termo t�ttu tekivad ajuti ikkagi viited. Vaja muuta termo algoritmi: katkestusest v�lja v�i state machine.
; 11.4.: termomeetreid pollib n��d iga 10s tagant.
;---------------------------------------------------------
; 14.4.: uus prose - PIC18F46K80, alusta porteerimist...n��d ! Reload_side iga minule antud k�su puhul ehk nagu alguses oli.
;||256||R||device type (F1)||
;||257||R||firmware serial number||
;||258||R||serial number 1, LSB||
;||259||R||serial number 2, LSB||

; 25.4.: v�iks toimida - testimisele !
; 26.4.: fiksid, xyz juures v�tab kirjutamise algaadressi vastuse jaoks valesti ! SEivi k�su lugemisel !
; 27.4.: Dallas toimib kah. Test jooxeb l�bi - Neemele testida ! NB! Lisasin WDT perioodiga 4,2s.
; 29.4.: Vist ei jookse kokku nagu eelmine plaat. Ilma Dallaseta ei j�ta ka vastuseid vahele.
; ajutine j�nx PWRSW-ga initis.
;
;--------------------------- Uus prose ja LISAME ka juhitava PWMi f-naalsuse -------------------------------
;kui aeglase pwm tegemiseks laheb, siis tasuks ehk kohe ka selle variandiga arvestada, et saaks kergesti lyhikeste impulssidega servo pwm teha.
;siis peaks kestuse ja perioodi juhtimine k�ima 100ms asemel ehk 100 mikrosekundi kaupa. yhiku valik kanali v�i perioodi registri yhe bitiga.
;juhtregistrid v�iks aga nii globaalse perioodi kui kanalipulsside jaoks samad olla ja mingi vanem bit pulsi pikkkuse registris ytleks, mis yhikuid data 
;jaoks m�eldakse. servo pwm spetsialist ma ei ole, ehk helikopteritega seoses on sul rohkem aimu, mida seal vaja on.
;esialgu defineeriks ma sellised kanalid
;registriaadressid 108..107 DO MSB kanalite jaoks (tulevikus ehk 100..115?  ehk nii MSB kui LSB jaoks?) - pulsi pikkuse juhtimine, kus
; reg. 100...107 -> juhib ANA porti(reg.0 bitid 0...7), 108...115 -> digiporti (reg.0 bitid 8...15)
;  bit15 - korduv pidev vs yhekordne impulss, korduv=1
;  bit14 - kestuse yhik, 100ms vs 100us? aeglasem=1
;  bit13..12 - reserv
;  bit10 - pidevpwm impulside polaarsus (periood algab 0 voi 1-ga)
;  bit9...bit0 - impulsi kestus, kusjuures teatud koodid on erilised:
;    0 - pidev 0
;    1 - pidev 1
;    2...1022 impulsi kestus bit14 maaratud yhikutes
;    1023 - yhekordne inversioon senisele seisule
;globaalne perioodi register, aadress 150, kus
;  bit15 - uuestikaivituse globaalne luba=1
;  bit14 - kestuse yhik, 100ms vs 100us? aeglasem=1
;  bit13..12  - reserv
;  bit10- v�lise synkro luba, 1=luba DI bit 15 0->1 alusel
;  bit9...bit0 - perioodi kestus, lubatud koodid 3..1023
;v�line s�nkro t�hendaks seda, et mingi DI sisendi (naiteks DI bit15, kus katkestus oli) saaks kasutusele v�tta 50Hz synkrosisendina. 
;saaks mitmekanalilise dimmeri teha... perioodi kestus t�histaks aga nominaalset sagedust, 50 v�i 60 Hz jaoks, v�i ka hoopis 100 vs 120 jaoks (pole ju oluline, 
;kumb poolperiood parajasti k�imas). laevadele saaks pakkuda 400Hz seadmeid...
;lisaks v�lisele s�nkrosisendile peaks olema synkroniseerimise v�imalus modbus kaudu. uus perioodiarvestus peaks algama hetkel, kui reg 150 sisse midagi kirjutatakse.
;--------
;selline m�te tuli aga k�ll veel v�lise synkro kohta, et synkroimpulsi kasutamine ei peaks toimuma otse, katkestuse saamisel kohe loendeid nullistama tormates
;vaid l�bi digitaalse faasidetektori (PLL). see v�imaldaks tasahilju parandusi sisse viia ja synkroimpulsi ebat�psest formeerimisest tulenevate juhuslike h�irete 
;m�ju v�hendada (see viga on paljudel labastel dimmeritel).
;4 kanalit pwn on aga t�iesti piisav.
;PLL puhul saab vist yhe registriga hakkama. v�imalikud t�hendused:
;kood 0 - synkro puudub
;kood 1 - s�nkro sagedusele 50 Hz
;kood 2 - s�nkro sagedusele 60 Hz
;
; PLL sisu on loendi, mis tegeleb synkro silumisega (keskmistamisega) - kuna yksikud
; synkroimpulsid v�ivad saabuda hajusalt, h�irete ja anduri ebat�psuse t�ttu. selle 
; PLL loendi nullistumisi kasutataks siis impulsipikkustega tegelevate loendite juhtimisel. 
; see reversiivloendi oleks t�psuse huvides v�hemalt 10-bitine ja loeks algseisust nullini maha.
; nullij�udmise ja v�lise synkro saabumise hetke vahe m��detakse, t��deldakse ja liidetakse
; loendi alglaadimiseks kasutatavale algseisu numbrile. see t��tlus peaks kasutama proportsionaalset
; ja integreerivat vea arvestamist (k�ituma kui PI regulaator). juhtm�ju=p*e+i*sum(e), kus e on
; faasiviga impulssides ja sum(e) on kogutud viga. veale e kui synkroimpulsi esifrondi ja loendi
; nullij�udmise vahele tuleb igaks juhuks lisada veel yks seaditustegur (faasinihe), mis kompenseerib
; synkroanduri hilistumise (v�i ettej�udmise). tegurid p ja i valitakse katseliselt selliselt, et
; saavutada kiire (minimaalse �leviskega) mittev�nkuv j�releh��lestumine. optimumi leidmiseks on
; mitu metoodikat, sellega tegelen ise (tegurid peavad olema registrites, 1 bait kummalegi).
;
; muide, kuigi impulsi t�iteteguri etteandmisel saaks lihtsalt aja anda, nagu yksikimpulsside korral,
; oleks pideva pwm jaoks mugavam kas ainsa v�i teise variandina kasutada juhtimises t�itetegurit
; (siis ei s�ltuks pikkuse arvestus valitud perioodist). muid perioode peale 50 ja 60 Hz jaoks vajalike
; ei pea synkroniseerima, aga v�imalikud v�iks olla isegi kuni 1 tunnini ulatuvad perioodid (p�randak�tte juhtimine!).
;
; selle pll j�rgih��lestumise kirjeldamisega tekkiski juba �ks uus m�te - yks variant PWM t�iteteguri
; m��ramisel olla mitte lihtsalt t�iteteguri etteandmine, vaid selle asemel mingit soovitud ja tegeliku
; suuruse etteandmine, mille vahet PWM v�ljund sisemise PI algoritmi kaudu nulli yritaks ajada. see
; vabastaks juhtkontrolleri PI-kontuuride juhtsuuruste pidevast arvutamisest ja meelespidamisest.
; iga kanali jaoks vaja siis ka oma p ja i ette anda, milleks tuleks registreid reserveerida.
;
; 5.5.: pwmi reg-e lugemine/kirjutamine tehtud, vist �ige kah ;)
;================================================================
; setup_port ikka kummaline vist ? Uurida sealt alates !
; 12.5.: RCIE ja RCIF olid valedes registrites, fixed !
; 13.5.: termomeetrite n�idu kontroll: kui 1* 85C, ei usu. Kui 2* j�rjest siis usub. Eelnevast juba: kui >125 -> ei usu
; justku toimib. Lisaks kontrollitud PWMi reg-te lugemist, ok.
; 6.6.: v�tab vana plaadi koodist uptime-counteri koodi �le...(32-bitine uptime loendi reg. 498...499 (1F2, 1F3)) 
; teeme ka Dallase DS2438 kivi lugemise (kuni 9 kivi)
; F/w ver. 2.1
; 14.6.: reset on n��d register 999 ! Teeme DS2438 registrite lugemist.
; 16.6.: ADC juhtbitid olid valed, n��d Vref=4,1V ja muu kah �ige.
; 23.6.: tehtud DS2438 lugemine, ok. Kivi tahab eraldi toidet ! F/w 2.2. NB ! TXIF tuleb keelata !!!
; 1.7.: mitmed parandused stabiilsuse nimel. Testime.... F/w 2.3.
; 2.7.: seni elus !!! P�hiline asi oli "bsf		PIE1,RC1IE" kohas "aaa". Testida termoandurite lugemise �igsust. Tegin RCIE1 lubamise ka sidetaimeri �let�itumise juurde !
; kui stardil DS-de otc m�ttas, j��bki ootama - fiksida ka siis kui t�� ajal l�his tekib!
; seda ei viici fixida aga muu toimib. "Measure" juurde lisatud Taqu=1,5ms! F/w.: 2.4.
; 10.7.: "measure" juurde veel: kui ADC > 4095, ei salvesta!
; 14.7.: Ver. 2.5. Keskmistab k�iki ADC kanaleid, enne lahutab min ja max (�le 10 s�mpli). Taqu=277uS tagasi !
; NB ! incrpointer pole vajalik, v�iks v�lja visata! Kui viitcib...
; 4.8.: uus PWMi specs.
; 5.8.: T0 pulsi l�petamine kompuleerub juba. Seiv.
; 6.8.: k�skude viga fixed, vastab paremini, pulss tekib kuid periood vale !? T0int teha prioriteetseks (serial segab). Seiv.
; 8.8.: serint'i l�pus tuli sidetaimer seisata, solkis pika saadetise puhul l�bi katkestuse counteri ��.
; P��ab teha T0inti prioriteetse... Tehtud aga segab - hakkab pakette vahele j�tma
; 12.8.: PWM justqi t��tab aga perioodi kestus on 1 sammu v�rra liiga pikk ja pulsi kestus l�hike. SEiv.
; 12.8.: pulsi kestused �iged. Seiv. Testima Neemele. Ver.: 2.6 ikka veel.
; 13.8.: termomeetrite lugemise viga fiksed - seriali puhvri �rav�tmine nihutas lugemist. V.2.6 ikka !
; PWMi muutus kajastub ka registris 0. Register 149 kasutusele v�etud. Vist toimib ?! Ver. 2.7 !!!
; ID-de lugemine oli nihu -fixed. Edasi teeb wr_multi vaid PWMi reg-tele (100...115). Koopia siit!
; wr-multi ka olemas, koopia. Ver. 2.8 !
; PWM n��d hulka parem koos termodega. TEstimsele ! Ver. 2.9.
; 15.8.: chk_len asemel validate. V�ib p�hjustada clrsticky biti viga.. Fixed PWM'i juhtregistri 151 viga -> olid vahetuses. reset_pwm'is ka see viga. Testima. Ver.2.9. ikkagi.
; 20.8.: PWMi reset iga perioodi alguses EI tohi nullida faasiloendit! Fiksed. Ei ole nii, peab faasiloendi nullima. Viga mujal... :(
; 21.8.: viga oli selles et enne pulsi kestuse mahatixumist tuli kontrollida kas pulss �ldse alanud. Koopia !
; 22.8.: pulss l�peb siis kui on aeg mitte perioodi alguses. R0 ja pwm nagu seni, R151 on v�ljund ja r/o. Tehakse R0 ja PWMi XORina. Ver.: 2.10. Testima (perioodi kestus on vist veel veica vale (,25 us pikem !?)
; 24.8.: 1 pisuke viga fiksed...
; 25.8.: veel vigu fiksetud, PWM v�ix n��d toimida. Lisaks sooviti et R273 kirjutamisel tehtax kohe reset1. Vist on.. ? Ver 2.11
; 26.8.: perioodi alguses ikkagi ei tohtinud kestusi taastada. 1ms loendi ei ole v�ga t�pne, vist kuna kood aeglane. Testima...
; 27.8.: 1ms loendi ikkagi 4. Viga faasdnX kus Seero asemel kontrollliti ZERO't !. Neemele...
; 28.8.: faasup juures tuli enne l�litamist kontrollida, kas pulss �kki veel kestab. Testima...
; 1.9.: WDT reset oli 1h, n��d ca 1s. RESET k�sk vaid kui WR data DEAD. Discovery leiab DS2438d ka siis kui periood juba olemas (PEIE tuli keelata)
; 2.9.: ADC m��tmisi parandet, incrpointer v�lja. On parem k�ll !
; v�idetavalt ei vasta, minul vastab aga sitalt. WDT perioodi pikendada ? EI ! Viimase ADC knali tulemus kirjutas �le seriali puhvri 1.baidi. Mix???!!!???
; 4.9.: lubame 0 perioodi, siis toimivad �xixkpulsid. Testida !
; 9.9.: reset1 asemele TXi j�rgiv sisend mis tekitab Dir signaali. Dir l�peb CRC arvutamise alguses ja muidugi seriali resetis.
; ei simuleeru (katckestusi ei tule)
;===============================================================================
;=============================================================================== 
;*********************** Raudv�rk **********************************************
;*** Side ***
#define Dir				PORTC,5					; v�ljundpuhvri juhtimine, HIGH=rcv, LOW=trnsmt
#define TxD				PORTC,6					; seriali otc
#define RxD				PORTC,7					; seriali otc
#define RdRxD			PORTC,.2				; j�lgib CCPIF kaudu seriali sidet ja tekitab sellele Dir signaali
;*** sisendid ***
#define Ana0			Registertemp5,.0
#define Ana1			Registertemp5,.1
#define Ana2			Registertemp5,.2
#define Ana3			Registertemp5,.3
#define Ana4			Registertemp5,.4
#define Ana5			Registertemp5,.5
#define Ana6			Registertemp5,.6
#define Ana7			Registertemp5,.7
;#define Ana0			PORTA,.0				; analoogsisend 1
;#define Ana1			PORTA,.1				; analoogsisend 2
;#define Ana2			PORTA,.2				; analoogsisend 3
;#define Ana3			PORTA,.3				; analoogsisend 4
;#define Ana4			PORTA,.5				; analoogsisend 5
;#define Ana5			PORTE,.0				; analoogsisend 6
;#define Ana6			PORTE,.1				; analoogsisend 7
;#define Ana7			PORTE,.2				; analoogsisend 8

#define DinPort			PORTB
#define Din0			PORTB,.0				; digisisend 1
#define Din1			PORTB,.1				; digisisend 2
#define Din2			PORTB,.2				; digisisend 3
#define Din3			PORTB,.3				; digisisend 4
#define Din4			PORTB,.4				; digisisend 5
#define Din5			PORTB,.5				; digisisend 6
#define Din6			PORTB,.6				; digisisend 7
#define Din7			PORTB,.7				; digisisend 8

;*** v�ljundid ***
#define DoutPort		PORTD
#define Dout0			PORTD,.0				; digiv�ljund 1
#define Dout1			PORTD,.1				; digiv�ljund 2
#define Dout2			PORTD,.2				; digiv�ljund 3
#define Dout3			PORTD,.3				; digiv�ljund 4
#define Dout4			PORTD,.4				; digiv�ljund 5
#define Dout5			PORTD,.5				; digiv�ljund 6
#define Dout6			PORTD,.6				; digiv�ljund 7
#define Dout7			PORTD,.7				; digiv�ljund 8

;*** 1-Wire ***
#define	_1WSPU			PORTC,.0				; 0 aktiveerib tugeva pull-up'i 1-wire liinil
#define	_1WDAT			PORTC,.1				; 1-Wire dataotc
#define	TRIS_1WDAT		TRISC,.1				; 1-Wire data TRIS-pinn
; *** resetid ***
;#define	reset1			PORTC,.2				; 3,3V reseti otc
#define	n_reset1		PORTC,.3				; 3,3V reseti inversioon-otc. Viimases versioonis sooviti uut inversiooni !
#define PWRSW			PORTC,.4				; mobla toitel�liti
;*** Wiegand ***
#define D0A				PORTB,.7				; A (0)-lugeja, siis digi-in ei funksi !
#define D1A				PORTB,.6
#define D0B				PORTB,.5				; B (1)-lugeja
#define D1B				PORTB,.4
; ********************** constantsid *******************************************
;*** rauda puutuv **
;*** A/D muundi kanalid ***
#define chan1			0x01					; ADC kanal 0
#define chan2			0x05
#define chan3			0x09
#define chan4			0x0D
#define chan5			0x11
#define chan6			0x15
#define chan7			0x19
#define chan8			0x1D
#define	sampletime		0xFF
;#define	sampletime1		0x20
;****** debounce ***
#define debtime			.1						; 10ms debounce aega igale sisendile
;****** side *******
#define RSBufLen			.99;				; saate-/vastuv�tu puhver
;****** k�sud *****
#define modbus_rd			0x03				; k�sk: loe registrit
#define modbus_wr			0x06				; k�sk: kirjuta registrisse
#define modbus_wrmulti		0x10				; k�sk: kirjuta mitu registrit korraga
#define modbus_cnf			0x45				; k�sk: kirjuta konfiregistrisse
#define IllFunc				0x01				; veakood: keelatud k�sk
#define IllDAdr				0x02				; veakood: vale registri aadress
#define maxregisters		.255;100+.170+.4				; niimitu 2-baidist registrit seadmel on
#define workregisters		.19+.17;0					; niimitu t��registrit on, muud on k�rgema adrega ja s�steemi setupi jaoks
#define sidetime			.30					; 30 s m��dudes kui ei laeku arusaadavaid k�sk, v�etakse default sideparameetrid
;****** taimerid, ajad ***
#define	serialtime			0x06				; 30 ms k�su laekumise aega alates 1. baidist
#define sekundiaeg			.100
#define _10sekundiaeg		.1;0
#define	T1resoL				0x7F				; 10ms aeg @ 11.0592 MHz
#define	T1resoH				0xF2					
#define	T3resoL				0x92;7F				; 5,5ms aeg @ 11.0592 MHz
#define	T3resoH				0xF8;F2					
#define	T0reso				.214				; 250 us aeg @ 11.0592 MHz
#define senstime			0x02;1				; 5 ms debounce aega
#define defaultserial		0x0A
#define	wiegandtime			.3;10					; 100 ms max ooteaega (viimasest pulsist loetuna)
; b0,1,2 - kiirus, default 19200 (010)
; b3 -debounce k�igile sisenditele, default  ON
; b4 - sticky bitt k�igile sisenditele, default OFF
; b5,6 - ei kasuta
; b7 - paarsuse bitt, (0= even parity), default EVEN
;****** Dallase kr�pp *******
#define MaxDS1820Sensors	.9*.2				; rohkem ei mahu m�llu ��...
#define MaxDS2438Sensors	.9*.2				; rohkem ei mahu m�llu ��...
#define DS1820ID		0x28					; DS18B20 family code
#define DS2438ID		0x26					; DS2438 family code
#define	ConvertT		0x44					; k�sk: m��da temperatuuri (m�lematele anduritele)
#define	ConvertV		0xB4					; k�sk: m��da (pinget DS2438-le)
#define	SkipRom			0xCC					; k�sk: ID-ROMi lugemine vahele 
#define	MatchRom		0x55					; k�sk: loe vaid konkreetset andurit
#define	ReadScratch		0xBE					; k�sk: loe temp. ja muu scratchpadilst
#define	RecallMem		0xB8					; k�sk: kirjuta scratchpadi tulemused
#define WrScr			0x4E					; k�sk: kirjuta scratchpad'i
#define _VddConnected	0x09
#define _VadcConnected	0x01
;************ andurid ************************
#define Akanaleid			.8					; niimitu kanalit on rauas olemas
#define m��tmisi			.10					; 10 m��tmist, neist min ja max lendavad v�lja
#define MinInitialH			HIGH (.4096)
#define MinInitialL			LOW (.4096)
#define MaxInitialH			HIGH (.0)
#define MaxInitialL			LOW (.0)
;***** Lipukesed ******
#define	d0timeron			dinflags,.0			; Din sisendite debounce taimerid
#define	d1timeron			dinflags,.1
#define	d2timeron			dinflags,.2
#define	d3timeron			dinflags,.3
#define	d4timeron			dinflags,.4
#define	d5timeron			dinflags,.5
#define	d6timeron			dinflags,.6
#define	d7timeron			dinflags,.7

#define	a0timeron			ainflags,.0			; Ain sisendite debounce taimerid
#define	a1timeron			ainflags,.1
#define	a2timeron			ainflags,.2
#define	a3timeron			ainflags,.3
#define	a4timeron			ainflags,.4
#define	a5timeron			ainflags,.5
#define	a6timeron			ainflags,.6
#define	a7timeron			ainflags,.7

#define dinstuck0			dinsticky,.0		; digisisendite sticky bitid
#define dinstuck1			dinsticky,.1		
#define dinstuck2			dinsticky,.2		
#define dinstuck3			dinsticky,.3		
#define dinstuck4			dinsticky,.4		
#define dinstuck5			dinsticky,.5		
#define dinstuck6			dinsticky,.6		
#define dinstuck7			dinsticky,.7		

#define ainstuck0			ainsticky,.0		; analoog-digisisendite sticky bitid
#define ainstuck1			ainsticky,.1		
#define ainstuck2			ainsticky,.2		
#define ainstuck3			ainsticky,.3		
#define ainstuck4			ainsticky,.4		
#define ainstuck5			ainsticky,.5		
#define ainstuck6			ainsticky,.6		
#define ainstuck7			ainsticky,.7		

#define dinpress0			dinpress,.0			; digisisendite ajutised bitid
#define dinpress1			dinpress,.1			
#define dinpress2			dinpress,.2			
#define dinpress3			dinpress,.3			
#define dinpress4			dinpress,.4			
#define dinpress5			dinpress,.5			
#define dinpress6			dinpress,.6			
#define dinpress7			dinpress,.7			

#define ainpress0			ainpress,.0			; analoogsisenaite ajutisea bitia
#define ainpress1			ainpress,.1			
#define ainpress2			ainpress,.2			
#define ainpress3			ainpress,.3			
#define ainpress4			ainpress,.4			
#define ainpress5			ainpress,.5			
#define ainpress6			ainpress,.6			
#define ainpress7			ainpress,.7		

#define sens1tmron			senstmrs,.0
#define sens2tmron			senstmrs,.1
#define sens3tmron			senstmrs,.2
#define sens4tmron			senstmrs,.3
#define sens5tmron			senstmrs,.4
#define sens6tmron			senstmrs,.5
#define sens7tmron			senstmrs,.6
#define sens8tmron			senstmrs,.7

#define reset1ena			flags,.0			; reset1 aeg t�is tiksunud (peale pingestamist) ja n��d saab pulssi anda kui vaja)
#define reset2ena			flags,.1			; reset2 aeg t�is tiksunud (peale pingestamist) ja n��d saab pulssi anda kui vaja)
#define reset1pulseon		flags,.2			; reseti 1 pulsi kestuse taimer k�ib
#define reset2pulseon		flags,.3			; reseti 2 pulsi kestuse taimer k�ib
#define wiegandAto			flags,.4		
#define wiegandAtmron		flags,.5		
#define	wiegandBto			flags,.6
#define wiegandBtmron		flags,.7

#define reg10				flags1,.0			; loeti registrit 10: nulli ta sisu (wiegandi bitikaunt)
#define dallas				flags1,.1
#define temper				flags1,.2
#define cmd_ok				flags1,.3			; saadi k�sk modbusilt
#define loendi				flags1,.4
#define	pwm					flags1,.5			; p��rdumine pwmi registrite poole
#define sync				flags1,.6			; kirjutati reg-e 150 mille peale syngitaxe PWMi
#define nomoreds2438		flags1,.7
#define Seero				flags1,.7			; PWMi pulsi l�petamise juures

#define suurem				Measflags,.0
#define vaiksem				Measflags,.1
#define vordne				Measflags,.2
#define res_pwm				Measflags,.3
#define	write				Measflags,.4
#define	broadcast			Measflags,.5

;**** 1-wire ****
#define DallasState			bits
;=============================================================================== 
;*********************** Raudv�rk **********************************************
	include "P18F46K80.inc"  	
;**** Prose konfi (ropud :) s�nad ****
;Program Configuration Register 1H
		CONFIG	 FOSC = HS1				; medium power osc
		CONFIG	 RETEN = OFF			; ULP regulator OFF
		CONFIG	 INTOSCSEL = HIGH 		; in Low-power mode during Sleep
		CONFIG	 SOSCSEL = DIG         	; Digital (SCLKI) mode
		CONFIG	 XINST = OFF          	; Disabled
		CONFIG	 PLLCFG = OFF         	; Disabled
		CONFIG	 FCMEN = OFF          	; Disabled
		CONFIG	 IESO = OFF          	; Disabled
		CONFIG	 PWRTEN = ON          	; Enabled
		CONFIG	 BOREN = SBORDIS      	; Enabled in hardware, SBOREN disabled
		CONFIG	 BORV = 0	          	; 3.0V
		CONFIG	 BORPWR = LOW         	; BORMV set to low power level
;		CONFIG	 WDTEN = OFF          	; WDT disabled in hardware; SWDTEN bit disabled
		CONFIG	 WDTPS = 256		   	; 1:16 ehk ca 1,023 s
		CONFIG	 WDTEN = SWDTDIS        ; WDT enabled in hardware; SWDTEN bit disabled
		CONFIG	 MCLRE = OFF          	; MCLR Disabled, RG5 Enabled
		CONFIG	 STVREN = OFF         	; Disabled
        CONFIG	 CP0 = ON     
        CONFIG	 CP1 = ON     
        CONFIG	 CP2 = ON     
        CONFIG	 CP3 = ON     
        CONFIG	 CPB = ON     
		CONFIG	 WRT0 = ON   
		CONFIG	 WRT1 = ON   
		CONFIG	 WRT2 = ON   
		CONFIG	 WRT3 = ON   
		CONFIG	 WRTC = ON   
		CONFIG	 WRTB = ON   
				errorlevel -302					; Compaileri vingumised �ra
				errorlevel -305
				errorlevel -306 

; ************** M�lu jaotus **********	    	
				cblock 0x00;20
					sidetaimer					; m��dab 30s side kadumise aega
					sekunditmr
					lastinputs					; PORTB sisendite eelmine seis
					flags
					flags1
					sampledly					; m��tmise Taqu viide
					meascount					; analoogpingete keskmistamiste loendi
					Measflags					; min/max v�rdlemise lipukesed
					serpartmp
; *** resettide taimerid ****
					reset1strttmrH
					reset1strttmrL
					reset1pulsetmr
					reset1dlytmr
					reset2strttmrH
					reset2strttmrL
					reset2pulsetmr
					reset2dlytmr
; *** k�su parseldamine jne ***
					countH
					countL						; ka Dallase otsimise rutiinis !
					adrtemp
; *** serial ***
					Char 			          	; vastuv�etud s�mbol, saatmisel sendtemp'i asemel
					bytecnt
					Flags	
					serialtimer					; seriali taimer (40mS ja siis VIGA kui k�sk ikka poolik)
; *** CHK arvutus ***
					mb_del1
					mb_temp1					; kasutusel ka comapre_1616's !!!
					mb_temp2
					_RS485chkH
					_RS485chk
; *** debounce, sisendid ****
					dinflags
					ainflags
					dinpress
					ainpress
					din0tmr
					din1tmr
					din2tmr
					din3tmr
					din4tmr
					din5tmr
					din6tmr
					din7tmr
					ain0tmr
					ain1tmr
					ain2tmr
					ain3tmr
					ain4tmr
					ain5tmr
					ain6tmr
					ain7tmr
; *** sisendite sticky bitid ****
					dinsticky
					ainsticky
; *** kirjutamise abimuutujad ***
					datatemp
; *** Registrite kirjutamise abi ***
					Registertemp5
					Registertemp7				; ajutine register modb_writes ja setup_pordis
; *** loendite stuff ***
					muutus
					senstmrs
; *** Wiegand ja loendite taimerid ***
					wiegandAtimer
					wiegandBtimer
					dataport					
					WAbitcount
					WBbitcount
					Loendi1tmr
					Loendi2tmr
					Loendi3tmr
					Loendi4tmr
					Loendi5tmr
					Loendi6tmr
					Loendi7tmr
					Loendi8tmr				
; *** 1-wire ***
					OneWireByte					; 0x44
					TEMP0						; 0x45
					TEMP1						; 0x46
					TEMP2						; 0x47
; Work areas
					DS2438found
					rombit_idx					; 0x48	ka comparemin/max sub'ides
					bits						; 0x49
					DS1820found					; 0x4A - leitud Dallase andurite arv
					curr_discrep				; 0x4B - ka sub1616min/max rutiinis !
					last_discrep				; 0x4C
					done_flag					; 0x4D
					return_value				; 0x4E
; One-Wire work area
					work0                       ; CRC (8 bits) 0x4F
					work1                       ; Serial # (48 bits) 
					work2                       ; " 				
					work3                       ; "
					work4                       ; "						
					work5                       ; "
					work6                       ; "
					work7                       ; Family Code (8 bits) (56)
					_10sekunditmr
					_1mscount					; PWMi sammu arvutamisel
; *** konteksti seivimine ***
					W_Temp						; 0x57 - konteksti seivimine katckestuse ajaks
					S_Temp						; 0x58
					FSRtmpH						; 0x59
					FSRtmpL						; 0x5A
					FSR1tmpH					
					FSR1tmpL								
					W_Temp1						; konteksti seivimine prioriteetse katckestuse ajaks
					S_Temp1						
					FSRtmpH1			
					FSRtmpL1	
; ===== m�lupank 2 =====
					phasecounter				; jooksva perioodi number
					RegX:.2
					AbiPuhver:.61		        ; esialgu vaba
; **** pwmi kirjutamise abi
					Register149in:.2			; Viide ms kuni j�rgmise perioodi alguseni
					Register149out:.2			; Aeg ms jooksva perioodi algusest
					pwmtmp:.3
; ===== m�lupank 3 =====
; *** registrid ***
					Register0:.2				; digiv�ljundid (MSB) ja UIO (LSB), r/w	 		0xAB,AC NB! +1  siitmaalt alates
					Register1:.2				; digisisendid (MSB) ja UIO (LSB), r	  		0xAD,AE
					Register2:.2				; analoogsisendid ADC chan 1 (0) tulemus, r		0xAF,B0
					Register3:.2
					Register4:.2
					Register5:.2
					Register6:.2
					Register7:.2
					Register8:.2
					Register9:.2
					Register10:.2				; HIGH - Wiegand A bittide hulk, LOW - B kohta	0xBF,C0 NB! +1  siiani
					Register11:.2				; Wiegand A bitijada
					Register12:.2				
					Register13:.2				
					Register14:.2				;												0xC8,C9
					Register15:.2				; Wiegand B bitijada
					Register16:.2				
					Register17:.2			
					Register18:.2				;												0xD0,D1
; *** setup ****
					Register256:.2				; r, device type (0xF1)							0xD2,D3
					Register257:.2				; r, f/w number									0xD4,D5
					Register258:.2				; r, serial, 1. osa (130 dec)					0xD6,D7
					Register259:.2				; r, serial 2.osa (1...dec)						0xD8,DA
					Register270:.2				; r/w, ADC Vref bitid 3,4,5 nagu PDF-is kirjas	0xDA,DB
					Register271:.2				; r/w, UIO analoog/digitaalseis, 1=digisisend	0xDC,DD 			NB! Vanem bait (271+.0) on XOR-i register !!!
					Register272:.2				; r/w,  pull-upp'ide seis ehk DO seis stardil, 1= PU sees 0xDE,DF
					Register273:.2				; r/w, seriali parameetrid						0xE0,E1
					Register274:.2				; r/w,  modbus'i aadress						0xE2,E3
					Register275:.2				; analoogpordi UIO suund, 1= v�ljund, 0= sisend	0xE4,E5
					Register276:.2				; r/w reset 1 toite pealetulemise kaitse aeg	0xE6,E7
					Register277:.2				; r/w reset 1 pulsi kestus (MSB) ja rakendumise aeg (LSB) 0xE8,E9
					Register278:.2				; r/w reset 2 toite pealetulemise kaitse aeg	0xEA,EB
					Register279:.2				; r/w reset 2 pulsi kestus (MSB) ja rakendumise aeg (LSB) 0xEC,ED
;					Register999					; teeb reseti !
; *** temperatuuri kontroll ***
					DallasChk:.9				; kuni FF !!
 					DS2438Chk:.9
				endc
; ===== m�lupank 4 =====
; *** loendid ja Dallase kr�pp ;) ***
				cblock 0x190					
; *** loendid ***
					Loendi1:.4					; DIN 1 pulsside loendi  Reg. 400,401
					Loendi2:.4					; 402, 403
					Loendi3:.4					; 404, 405
					Loendi4:.4					; 406, 407
					Loendi5:.4					; 408, 409
					Loendi6:.4					; 410, 411
					Loendi7:.4					; 412, 413
					Loendi8:.4					; 414, 415.
					Dallas1:.2					; Dallase termomeetrite n�idud (Reg. 600 -> 47)
					Dallas2:.2					; 601 -> 48
					Dallas3:.2					; 602 -> 49
					Dallas4:.2					; 603 -> 50
					Dallas5:.2					; 604 -> 51
					Dallas6:.2					; 605 -> 52
					Dallas7:.2					; 606 -> 53
					Dallas8:.2					; 607 -> 54
					Dallas9:.2					; 608 -> 55
					Dallas1wa:.8				; Dallase termomeetrite ID-d (1A2: Reg. 650, 651, 652, 653 )
					Dallas2wa:.8				; 1A6: 654, 655, 656, 657
					Dallas3wa:.8				; 1AA: 658, 659, 660, 661
					Dallas4wa:.8				; 662, 663, 664, 665
					Dallas5wa:.8				; 666, 667, 668, 669
					Dallas6wa:.8				; 670, 671, 672, 673
					Dallas7wa:.8				; 674, 675, 676, 677
					Dallas8wa:.8				; 678, 679, 680, 681
					Dallas9wa:.8				; 682, 683, 684, 685	
; *** uptime loendi ***
					LoendiUP:.4					; 498, 499
									endc
; *** PWMi registrid ***
					cblock	0x300
; bit15 � perioodiliselt korduv (1) v�i �hekordne (0) impulss, korduv=1
; bit14 � impulsi sidumine perioodi loendiga
; bit13..12 � faasiga sidumine, N*90 deg, N=1..3
; bit11..0 - impulsi kestus ms
; 2...4090 impulsi kestus ms
					pwm0set:.2					; register 100 (0x64)
					pwm1set:.2
					pwm2set:.2
					pwm3set:.2
					pwm4set:.2
					pwm5set:.2
					pwm6set:.2
					pwm7set:.2
					pwm8set:.2
					pwm9set:.2
					pwm10set:.2
					pwm11set:.2
					pwm12set:.2
					pwm13set:.2
					pwm14set:.2
					pwm15set:.2					; register 115 (0x73)

;					Register149:.2				; kirjutamisel viide ms kuni j�rgmise perioodi alguseni. Lugemisel aeg ms jooksva perioodi algusest

; globaalne perioodi register, aadress 150, kus
;  bit15 - uuestikaivituse globaalne luba=1
;  bit14 - kestuse yhik, 100ms vs 100us? aeglasem=1
;  bit13..12  - reserv
;  bit10- v�lise synkro luba, 1=luba DI bit 15 0->1 alusel
;  bit9...bit0 - perioodi kestus, lubatud koodid 3..1023
; lisaks v�lisele s�nkrosisendile peaks olema synkroniseerimise v�imalus modbus kaudu.
; uus perioodiarvestus peaks algama hetkel, kui reg 150 sisse midagi kirjutatakse.
					Register150:.2				; register 150 PWMi periood 3�65535 ms (0x96)  ---> 0x320
; kuidas m��ratakse, kas antud v�ljund on PWMiga v�i tavav�ljund/sisend
; tekitasin selleks eraldi bitmap registri, aadress naiteks 151, bitid 0-15
					Register151:.2				; register 151. Bitt =1 siis antud pin on PWMiga h�ivatud
; *** PWMi t��registrid ****
					pwm0work:.2
					pwm1work:.2
					pwm2work:.2
					pwm3work:.2
					pwm4work:.2
					pwm5work:.2
					pwm6work:.2
					pwm7work:.2
					pwm8work:.2
					pwm9work:.2
					pwm10work:.2
					pwm11work:.2
					pwm12work:.2
					pwm13work:.2
					pwm14work:.2
					pwm15work:.2
;					phasecounter				; jooksva perioodi number
					periodtick:.3				; �hes perioodis on periood*4 sammu (250 us)
					masterpertick:.3			; sama aga siin numbrit hoitakse ja siit laetakse "periodtick'i"
					mphasetick:.2				; 1-s faasis t�pselt 1*periood sammu
					phasetick:.2				; t��register
;					BSRtmp
					endc
; *** DS2438 registrid ***
					cblock	0x400				; Vadc, I, Vdd, Temp.
					DS2438_1:.8					; Dallase DS2438 kivi n�idud: Reg. 700-2BC:(0x400,1), 701-2BD:(0x402,3), 702-2BE:(0x404,5), 703-2BF:(0x406,7)
					DS2438_2:.8					; Reg. 700, 701, 702, 703 (nr. 0x2BC...)
					DS2438_3:.8					; Reg. 704, 705, 706, 707 (0x2C0...2C3)-> 74 ?	
					DS2438_4:.8					; Reg. 708, 709, 710, 711 (0x2C4...)-> 74 ?	
					DS2438_5:.8					; Reg. 712, 713, 714, 715 (0x2C8...)-> 74 ?	
					DS2438_6:.8					; Reg. 716, 717, 718, 719 (0x2CC...)-> 74 ?	
					DS2438_7:.8					; Reg. 720, 721, 722, 723 (0x2D0...)-> 74 ?	
					DS2438_8:.8					; Reg. 724, 725, 726, 727 (0x2D4...)-> 74 ?	
					DS2438_9:.8					; Reg. 728, 729, 730, 731 (0x2D8...2DB)-> 74 ?	
					DS24381wa:.8				; Dallase DS2438 kivide ID-d (Reg. 750-0x2EE:(0x448,9); 751:(0x44A,B); 752:(0x44C,D); 753:(0x44E,F); ->  dec., 0x2EE ... hex)
					DS24382wa:.8				; Reg. 754, 755, 756, 757 ->  dec  450..457
					DS24383wa:.8				; Reg. 758, 759, 760, 761 ->  dec  458..45F
					DS24384wa:.8				; Reg. 762, 763, 764, 765 ->  dec
					DS24385wa:.8				; Reg. 766, 767, 768, 769 ->  dec 46F
					DS24386wa:.8				; Reg. 770, 771, 772, 773 ->  dec
					DS24387wa:.8				; Reg. 774, 775, 776, 777 ->  dec 47F
					DS24388wa:.8				; Reg. 778, 779, 780, 781 ->  dec
					DS24389wa:.8				; Reg. 782, 783, 784, 785 ->  dec 48F
; *** analoogpingete m��tmine ja keskmistamine ***
					ch1meascnt
					ch1sum:.2
					ch1min:.2
					ch1max:.2
					ch2meascnt
					ch2sum:.2
					ch2min:.2
					ch2max:.2
					ch3meascnt
					ch3sum:.2
					ch3min:.2
					ch3max:.2
					ch4meascnt
					ch4sum:.2
					ch4min:.2
					ch4max:.2
					ch5meascnt
					ch5sum:.2
					ch5min:.2
					ch5max:.2
					ch6meascnt
					ch6sum:.2
					ch6min:.2
					ch6max:.2
					ch7meascnt
					ch7sum:.2
					ch7min:.2
					ch7max:.2
					ch8meascnt
					ch8sum:.2
					ch8min:.2
					ch8max:.2
				jurajee:.1
;****** side *******
					Puhver:RSBufLen		        ; seriali puhver, kuni 99 baiti 

					endc

 ; ************* sub1616 rutiini asjad ****************	    	
#define			DestH			curr_discrep		
#define			DestL			last_discrep		
#define			SourceH			done_flag		
#define			SourceL			return_value		
; ************* sub comapre_1616 asjad ****************
#define 		XH				mb_temp1
#define 		XL				mb_temp2
#define 		YH				_RS485chkH
#define 		YL				_RS485chk
 ; ************* asjad puhvris peale k�su vastuv�ttu ****************	    	
;#define			m_adr			Puhver+.0		; slave aadress
;#define			m_cmd			Puhver+.1		; k�sukood
#define			m_radrH			Puhver+.2		; loetava/kirjutatava registri aadress HIGH
#define			m_radrL			Puhver+.3		; loetava/kirjutatava registri aadress LOW
#define			n_regH			Puhver+.4		; loetavate/kirjutatavate registrite arv HIGH
#define			n_regL			Puhver+.5		; loetavate/kirjutatavate registrite arv LOW
; **** lipukesed ****
#define 		SerialTimerOn 	Flags,.0		; seriali vastuv�tu taimer s/v
#define			cmd10			Flags,.1		
#define			SerialTimeOut 	Flags,.2
#define 		readonly		Flags,.3		; kui register on read-only
#define 		sidetmron		Flags,.4
#define			writeit			Flags,.5
#define			clrsticky		Flags,.6		; kui loeti registrit 1 suvalises kombinatsioonis teistega
#define			reg0			Flags,.7

#define 		in_sticky		Register273+.1,.4
#define 		in_debounce		Register273+.1,.3
; **** serial port ****

; **** Prose lipukesed ****
#define CARRY           	STATUS,C 
#define ZERO            	STATUS,Z 
; *****************************	    	
   	

  
				org	0x000
   				goto	main
;===============================================================================
;*********************** katckestused ******************************************
;===============================================================================
				org     0x0008					; Ints HIGH
				btfsc	PIR3,CCP2IF				; j�rgitava serial-signaali katckestus?
				call	LisaSer
				retfie                   	

				org 	0x0018					; Ints LOW
_Push:
	    		movwf   W_Temp           		; Seivi kontekst        
				swapf   STATUS,W         
				movwf   S_Temp   
				movff	FSR0L,FSRtmpL
				movff	FSR0H,FSRtmpH
				banksel	.0
				btfsc	INTCON,RBIF				; kas mingi Wiegandi daataots v�i digisisend (loendi) liigutas ennast?
				call	Loendid					; jepp, vaata kumb oli
				btfsc	INTCON,TMR0IF			; PWMi taimeri (TMR0) katckestus?
				call	T0int
 				btfsc	PIR1,TMR1IF				; Taimer 1'e katckestus?
				goto	T1int
Int_2:			btfsc	PIR1,RC1IF				; Seriali katckestus?
				call	SerInt					
_Pop:			movff	FSRtmpL,FSR0L
				movff	FSRtmpH,FSR0H
				swapf   S_Temp,W         
				movwf   STATUS           		; taasta STATUS         
				swapf   W_Temp,F         
				swapf   W_Temp,W         		; Taasta W         
				retfie                   	
;===============================================================================
; ******* CCP2 INT. *******
;===============================================================================
LisaSer:		bsf		Dir						; Dir signaal algab
				bcf		PIR3,CCP2IF
				bcf		PIE3,CCP2IE
				retfie                   	
;===============================================================================
; ******* PWMi taimeri INT. *******
;===============================================================================
T0int:			bcf		INTCON,TMR0IF
				bcf		T0CON,TMR0ON			; taimer stopp kuniks tuunime
				movlw	T0reso					; lae taimer 0 (katkestus iga 250 uS tagant)
				movwf	TMR0L
				bsf		T0CON,TMR0ON			; taxo taas tixuma
				movff	FSR1H,FSR1tmpH			; seivi FSR1
				movff	FSR1L,FSR1tmpL
				movf	Register149in+.0,W		; kas peab viivitama enne perioodiga alustamist?
				addlw	.0
				btfss	ZERO
				goto	T0int0					; peab !
				movf	Register149in+.1,W		
				addlw	.0
				btfss	ZERO
				goto	T0int0	
				goto	T0int1					; ei pea (enam) !				
T0int0:			decfsz	_1mscount				; 1 ms loendi
				goto	T0intend				
				movlw	.4						; jaap, taasta 1ms loendi
				movwf	_1mscount
				decf	Register149in+.1,F
				btfsc	CARRY
				goto	T0int2
				bcf		CARRY
				decf	Register149in+.0,F
T0int2:			movf	Register149in+.0,W		; kas ikka veel peab viivitama ?
				addlw	.0
				btfss	ZERO
				goto	T0intend				; ikka peab ...
				movf	Register149in+.1,W		
				addlw	.0
				btfss	ZERO
				goto	T0intend					
; jobutamine l�bi !!!
T0int1:			movff	periodtick+.0,WREG		; periood juba l�bi ?
				addlw	.0
				btfss	ZERO
				goto	decpertick				; eip
				movff	periodtick+.1,WREG
				addlw	.0
				btfss	ZERO
				goto	decpertick				; eip
				movff	periodtick+.2,WREG
				addlw	.0
				btfss	ZERO
				goto	decpertick				; eip
T0intfaas:		call	reset_pwm				; taasta perioodi ja faasi kestused. Pulsside kestuseid mitte !
OutHigh:		; hakkame vastavalt vajadusele sobivaid v�ljundeid k�rgex t�stma 
Faasup0:		LFSR	.0,Register151+.1		; siia l�heb kirja v�ljundi reaalne seis
				LFSR	.1,pwm0work+.0
				call	chkzero					; kas kestus on 0 ?
				btfsc	Seero
				goto	Faasup1					; jah, v�ta j�rgmine v�ljund
				btfss	INDF1,.6				; l�litame vaid sobival faasil ?
				goto	Faasup0nf				; eip, v�ib kohe l�litada
				swapf	INDF1,W					; jah, kas on �ige faas ?
				andlw	0x03
				subwf	phasecounter,W		
				btfss	ZERO
				goto	Faasup1					; ei, v�ta j�rgmine v�ljund
Faasup0nf:				btfsc	RegX+.1,.0				; v�ljund l�heb k�rgeks, kas juba on ?
				goto	Faasup1					; jah, v�ta j�rgmine v�ljund
		btfss	Register0+.1,.0			; v�ljundi m��ramiseks tee XOR reg-ga D0
				goto	Faasup0_0				; D0 on 0 -> v�ljund = 1
;				btfsc	RegX+.1,.0				; v�ljund l�heb k�rgeks, kas juba on ?
;				goto	Faasup1					; jah, v�ta j�rgmine v�ljund
				bcf		PORTA,.0				
				bcf		INDF0,.0				; kajasta ka v�ljundite seisu r/o registris	151
				goto	Faasup0ex				; tehtud !
Faasup0_0:		bsf		PORTA,.0				
				bsf		INDF0,.0				; kajasta ka v�ljundite seisu r/o registris	151
Faasup0ex:		movff	pwm0set+.1,pwm0work+.1	; loe pulsi kestus (tarbetu ?)
				movff	pwm0set+.0,pwm0work+.0
				bsf		RegX+.1,.0
Faasup1:		LFSR	.1,pwm1work+.0
				call	chkzero					; kas kestus on 0 ?
				btfsc	Seero
				goto	Faasup2					; jah, v�ta j�rgmine v�ljund
				btfss	INDF1,.6				; l�litame vaid sobival faasil ?
				goto	Faasup1nf				; eip, v�ib kohe l�litada
				swapf	INDF1,W					; jah, kas on �ige faas ?
				andlw	0x03
				subwf	phasecounter,W		
				btfss	ZERO
				goto	Faasup2					; ei, v�ta j�rgmine v�ljund
Faasup1nf:				btfsc	RegX+.1,.1				; v�ljund l�heb k�rgeks, kas juba on ?
				goto	Faasup2					; jah, v�ta j�rgmine v�ljund
		btfss	Register0+.1,.1			; v�ljundi m��ramiseks tee XOR reg-ga D0
				goto	Faasup1_0				; D0 on 0 -> v�ljund = 1
;				btfsc	RegX+.1,.1				; v�ljund l�heb k�rgeks, kas juba on ?
;				goto	Faasup2					; jah, v�ta j�rgmine v�ljund
				bcf		PORTA,.1				
				bcf		INDF0,.1				; kajasta ka v�ljundite seisu r/o registris	151
				goto	Faasup1ex				; tehtud !
Faasup1_0:		bsf		PORTA,.1				
				bsf		INDF0,.1				; kajasta ka v�ljundite seisu r/o registris	151
Faasup1ex:		movff	pwm1set+.1,pwm1work+.1	; loe pulsi kestus (tarbetu ?)
				movff	pwm1set+.0,pwm1work+.0
				bsf		RegX+.1,.1
Faasup2:		LFSR	.1,pwm2work+.0
				call	chkzero					; kas kestus on 0 ?
				btfsc	Seero
				goto	Faasup3					; jah, v�ta j�rgmine v�ljund
				btfss	INDF1,.6				; l�litame vaid sobival faasil ?
				goto	Faasup2nf				; eip, v�ib kohe l�litada
				swapf	INDF1,W					; jah, kas on �ige faas ?
				andlw	0x03
				subwf	phasecounter,W		
				btfss	ZERO
				goto	Faasup3					; ei, v�ta j�rgmine v�ljund
Faasup2nf:				btfsc	RegX+.1,.2				; v�ljund l�heb k�rgeks, kas juba on ?
				goto	Faasup3					; jah, v�ta j�rgmine v�ljund
		btfss	Register0+.1,.2			; v�ljundi m��ramiseks tee XOR reg-ga D0
				goto	Faasup2_0				; D0 on 0 -> v�ljund = 1
;				btfsc	RegX+.1,.2				; v�ljund l�heb k�rgeks, kas juba on ?
;				goto	Faasup3					; jah, v�ta j�rgmine v�ljund
				bcf		PORTA,.2				
				bcf		INDF0,.2				; kajasta ka v�ljundite seisu r/o registris	151
				goto	Faasup2ex				; tehtud !
Faasup2_0:		bsf		PORTA,.2				
				bsf		INDF0,.2				; kajasta ka v�ljundite seisu r/o registris	151
Faasup2ex:		movff	pwm2set+.1,pwm2work+.1	; loe pulsi kestus (tarbetu ?)
				movff	pwm2set+.0,pwm2work+.0
				bsf		RegX+.1,.2
Faasup3:		LFSR	.1,pwm3work+.0
				call	chkzero					; kas kestus on 0 ?
				btfsc	Seero
				goto	Faasup4					; jah, v�ta j�rgmine v�ljund
				btfss	INDF1,.6				; l�litame vaid sobival faasil ?
				goto	Faasup3nf				; eip, v�ib kohe l�litada
				swapf	INDF1,W					; jah, kas on �ige faas ?
				andlw	0x03
				subwf	phasecounter,W		
				btfss	ZERO
				goto	Faasup4					; ei, v�ta j�rgmine v�ljund
Faasup3nf:				btfsc	RegX+.1,.3				; v�ljund l�heb k�rgeks, kas juba on ?
				goto	Faasup4					; jah, v�ta j�rgmine v�ljund
		btfss	Register0+.1,.3			; v�ljundi m��ramiseks tee XOR reg-ga D0
				goto	Faasup3_0				; D0 on 0 -> v�ljund = 1
;				btfsc	RegX+.1,.3				; v�ljund l�heb k�rgeks, kas juba on ?
;				goto	Faasup4					; jah, v�ta j�rgmine v�ljund
				bcf		PORTA,.3				
				bcf		INDF0,.3				; kajasta ka v�ljundite seisu r/o registris	151
				goto	Faasup3ex				; tehtud !
Faasup3_0:		bsf		PORTA,.3				
				bsf		INDF0,.3				; kajasta ka v�ljundite seisu r/o registris	151
Faasup3ex:		movff	pwm3set+.1,pwm3work+.1	; loe pulsi kestus (tarbetu ?)
				movff	pwm3set+.0,pwm3work+.0
				bsf		RegX+.1,.3
Faasup4:		LFSR	.1,pwm4work+.0
				call	chkzero					; kas kestus on 0 ?
				btfsc	Seero
				goto	Faasup5					; jah, v�ta j�rgmine v�ljund
				btfss	INDF1,.6				; l�litame vaid sobival faasil ?
				goto	Faasup4nf				; eip, v�ib kohe l�litada
				swapf	INDF1,W					; jah, kas on �ige faas ?
				andlw	0x03
				subwf	phasecounter,W		
				btfss	ZERO
				goto	Faasup5					; ei, v�ta j�rgmine v�ljund
Faasup4nf:				btfsc	RegX+.1,.4				; v�ljund l�heb k�rgeks, kas juba on ?
				goto	Faasup5					; jah, v�ta j�rgmine v�ljund
		btfss	Register0+.1,.4			; v�ljundi m��ramiseks tee XOR reg-ga D0
				goto	Faasup4_0				; D0 on 0 -> v�ljund = 1
;				btfsc	RegX+.1,.4				; v�ljund l�heb k�rgeks, kas juba on ?
;				goto	Faasup5					; jah, v�ta j�rgmine v�ljund
				bcf		PORTA,.5				
				bcf		INDF0,.4				; kajasta ka v�ljundite seisu r/o registris	151
				goto	Faasup4ex				; tehtud !
Faasup4_0:		bsf		PORTA,.5				
				bsf		INDF0,.4				; kajasta ka v�ljundite seisu r/o registris	151
Faasup4ex:		movff	pwm4set+.1,pwm4work+.1	; loe pulsi kestus (tarbetu ?)
				movff	pwm4set+.0,pwm4work+.0
				bsf		RegX+.1,.4
Faasup5:		LFSR	.1,pwm5work+.0
				call	chkzero					; kas kestus on 0 ?
				btfsc	Seero
				goto	Faasup6					; jah, v�ta j�rgmine v�ljund
				btfss	INDF1,.6				; l�litame vaid sobival faasil ?
				goto	Faasup5nf				; eip, v�ib kohe l�litada
				swapf	INDF1,W					; jah, kas on �ige faas ?
				andlw	0x03
				subwf	phasecounter,W		
				btfss	ZERO
				goto	Faasup6					; ei, v�ta j�rgmine v�ljund
Faasup5nf:				btfsc	RegX+.1,.5				; v�ljund l�heb k�rgeks, kas juba on ?
				goto	Faasup6					; jah, v�ta j�rgmine v�ljund
		btfss	Register0+.1,.5			; v�ljundi m��ramiseks tee XOR reg-ga D0
				goto	Faasup5_0				; D0 on 0 -> v�ljund = 1
;				btfsc	RegX+.1,.5				; v�ljund l�heb k�rgeks, kas juba on ?
;				goto	Faasup6					; jah, v�ta j�rgmine v�ljund
				bcf		PORTE,.0				
				bcf		INDF0,.5				; kajasta ka v�ljundite seisu r/o registris	151
				goto	Faasup5ex				; tehtud !
Faasup5_0:		bsf		PORTE,.0				
				bsf		INDF0,.5				; kajasta ka v�ljundite seisu r/o registris	151
Faasup5ex:		movff	pwm5set+.1,pwm5work+.1	; loe pulsi kestus (tarbetu ?)
				movff	pwm5set+.0,pwm5work+.0
				bsf		RegX+.1,.5
Faasup6:		LFSR	.1,pwm6work+.0
				call	chkzero					; kas kestus on 0 ?
				btfsc	Seero
				goto	Faasup7					; jah, v�ta j�rgmine v�ljund
				btfss	INDF1,.6				; l�litame vaid sobival faasil ?
				goto	Faasup6nf				; eip, v�ib kohe l�litada
				swapf	INDF1,W					; jah, kas on �ige faas ?
				andlw	0x03
				subwf	phasecounter,W		
				btfss	ZERO
				goto	Faasup7					; ei, v�ta j�rgmine v�ljund
Faasup6nf:				btfsc	RegX+.1,.6				; v�ljund l�heb k�rgeks, kas juba on ?
				goto	Faasup7					; jah, v�ta j�rgmine v�ljund
		btfss	Register0+.1,.6			; v�ljundi m��ramiseks tee XOR reg-ga D0
				goto	Faasup6_0				; D0 on 0 -> v�ljund = 1
;				btfsc	RegX+.1,.6				; v�ljund l�heb k�rgeks, kas juba on ?
;				goto	Faasup7					; jah, v�ta j�rgmine v�ljund
				bcf		PORTE,.1				
				bcf		INDF0,.6				; kajasta ka v�ljundite seisu r/o registris	151
				goto	Faasup6ex				; tehtud !
Faasup6_0:		bsf		PORTE,.1				
				bsf		INDF0,.6				; kajasta ka v�ljundite seisu r/o registris	151
Faasup6ex:		movff	pwm6set+.1,pwm6work+.1	; loe pulsi kestus (tarbetu ?)
				movff	pwm6set+.0,pwm6work+.0
				bsf		RegX+.1,.6
Faasup7:		LFSR	.1,pwm7work+.0
				call	chkzero					; kas kestus on 0 ?
				btfsc	Seero
				goto	Faasup8					; jah, v�ta j�rgmine v�ljund
				btfss	INDF1,.6				; l�litame vaid sobival faasil ?
				goto	Faasup7nf				; eip, v�ib kohe l�litada
				swapf	INDF1,W					; jah, kas on �ige faas ?
				andlw	0x03
				subwf	phasecounter,W		
				btfss	ZERO
				goto	Faasup8					; ei, v�ta j�rgmine v�ljund
Faasup7nf:				btfsc	RegX+.1,.7				; v�ljund l�heb k�rgeks, kas juba on ?
				goto	Faasup8					; jah, v�ta j�rgmine v�ljund
		btfss	Register0+.1,.7			; v�ljundi m��ramiseks tee XOR reg-ga D0
				goto	Faasup7_0				; D0 on 0 -> v�ljund = 1
;				btfsc	RegX+.1,.7				; v�ljund l�heb k�rgeks, kas juba on ?
;				goto	Faasup8					; jah, v�ta j�rgmine v�ljund
				bcf		PORTE,.2				
				bcf		INDF0,.7				; kajasta ka v�ljundite seisu r/o registris	151
				goto	Faasup7ex				; tehtud !
Faasup7_0:		bsf		PORTE,.2				
				bsf		INDF0,.7				; kajasta ka v�ljundite seisu r/o registris	151
Faasup7ex:		movff	pwm7set+.1,pwm7work+.1	; loe pulsi kestus (tarbetu ?)
				movff	pwm7set+.0,pwm7work+.0
				bsf		RegX+.1,.7
; **** digiv�ljundite p�stiajamine ****
Faasup8:		LFSR	.0,Register151+.0		; siia l�heb kirja v�ljundi reaalne seis
				LFSR	.1,pwm8work+.0
				call	chkzero					; kas kestus on 0 ?
				btfsc	Seero
				goto	Faasup9					; jah, v�ta j�rgmine v�ljund
				btfss	INDF1,.6				; l�litame vaid sobival faasil ?
				goto	Faasup8nf				; eip, v�ib kohe l�litada
				swapf	INDF1,W					; jah, kas on �ige faas ?
				andlw	0x03
				subwf	phasecounter,W		
				btfss	ZERO
				goto	Faasup9					; ei, v�ta j�rgmine v�ljund
Faasup8nf:				btfsc	RegX+.0,.0				; v�ljund l�heb k�rgeks, kas juba on ?
				goto	Faasup9					; jah, v�ta j�rgmine v�ljund
		btfss	Register0+.0,.0			; v�ljundi m��ramiseks tee XOR reg-ga D0
				goto	Faasup8_0				; D0 on 0 -> v�ljund = 1
;				btfsc	RegX+.0,.0				; v�ljund l�heb k�rgeks, kas juba on ?
;				goto	Faasup9					; jah, v�ta j�rgmine v�ljund
				bcf		Dout0				
				bcf		INDF0,.0				; kajasta ka v�ljundite seisu r/o registris	151
				goto	Faasup8ex				; tehtud !
Faasup8_0:		bsf		Dout0				
				bsf		INDF0,.0				; kajasta ka v�ljundite seisu r/o registris	151
Faasup8ex:		movff	pwm8set+.1,pwm8work+.1	; loe pulsi kestus (tarbetu ?)
				movff	pwm8set+.0,pwm8work+.0
				bsf		RegX+.0,.0
Faasup9:		LFSR	.1,pwm9work+.0
				call	chkzero					; kas kestus on 0 ?
				btfsc	Seero
				goto	Faasup10				; jah, v�ta j�rgmine v�ljund
				btfss	INDF1,.6				; l�litame vaid sobival faasil ?
				goto	Faasup9nf				; eip, v�ib kohe l�litada
				swapf	INDF1,W					; jah, kas on �ige faas ?
				andlw	0x03
				subwf	phasecounter,W		
				btfss	ZERO
				goto	Faasup10				; ei, v�ta j�rgmine v�ljund
Faasup9nf:				btfsc	RegX+.0,.1				; v�ljund l�heb k�rgeks, kas juba on ?
				goto	Faasup10				; jah, v�ta j�rgmine v�ljund
		btfss	Register0+.0,.1			; v�ljundi m��ramiseks tee XOR reg-ga D0
				goto	Faasup9_0				; D0 on 0 -> v�ljund = 1
;				btfsc	RegX+.0,.1				; v�ljund l�heb k�rgeks, kas juba on ?
;				goto	Faasup10				; jah, v�ta j�rgmine v�ljund
				bcf		Dout1				
				bcf		INDF0,.1				; kajasta ka v�ljundite seisu r/o registris	151
				goto	Faasup9ex				; tehtud !
Faasup9_0:		bsf		Dout1				
				bsf		INDF0,.1				; kajasta ka v�ljundite seisu r/o registris	151
Faasup9ex:		movff	pwm9set+.1,pwm9work+.1	; loe pulsi kestus (tarbetu ?)
				movff	pwm9set+.0,pwm9work+.0
				bsf		RegX+.0,.1
Faasup10:		LFSR	.1,pwm10work+.0
				call	chkzero					; kas kestus on 0 ?
				btfsc	Seero
				goto	Faasup11				; jah, v�ta j�rgmine v�ljund
				btfss	INDF1,.6				; l�litame vaid sobival faasil ?
				goto	Faasup10nf				; eip, v�ib kohe l�litada
				swapf	INDF1,W					; jah, kas on �ige faas ?
				andlw	0x03
				subwf	phasecounter,W		
				btfss	ZERO
				goto	Faasup11				; ei, v�ta j�rgmine v�ljund
Faasup10nf:				btfsc	RegX+.0,.2				; v�ljund l�heb k�rgeks, kas juba on ?
				goto	Faasup11				; jah, v�ta j�rgmine v�ljund
		btfss	Register0+.0,.2			; v�ljundi m��ramiseks tee XOR reg-ga D0
				goto	Faasup10_0				; D0 on 0 -> v�ljund = 1
;				btfsc	RegX+.0,.2				; v�ljund l�heb k�rgeks, kas juba on ?
;				goto	Faasup11				; jah, v�ta j�rgmine v�ljund
				bcf		Dout2				
				bcf		INDF0,.2				; kajasta ka v�ljundite seisu r/o registris	151
				goto	Faasup10ex				; tehtud !
Faasup10_0:		bsf		Dout2				
				bsf		INDF0,.2				; kajasta ka v�ljundite seisu r/o registris	151
Faasup10ex:		movff	pwm10set+.1,pwm10work+.1; loe pulsi kestus (tarbetu ?)
				movff	pwm10set+.0,pwm10work+.0
				bsf		RegX+.0,.2
Faasup11:		LFSR	.1,pwm11work+.0
				call	chkzero					; kas kestus on 0 ?
				btfsc	Seero
				goto	Faasup12				; jah, v�ta j�rgmine v�ljund
				btfss	INDF1,.6				; l�litame vaid sobival faasil ?
				goto	Faasup11nf				; eip, v�ib kohe l�litada
				swapf	INDF1,W					; jah, kas on �ige faas ?
				andlw	0x03
				subwf	phasecounter,W		
				btfss	ZERO
				goto	Faasup12				; ei, v�ta j�rgmine v�ljund
Faasup11nf:				btfsc	RegX+.0,.3				; v�ljund l�heb k�rgeks, kas juba on ?
				goto	Faasup12				; jah, v�ta j�rgmine v�ljund
		btfss	Register0+.0,.3			; v�ljundi m��ramiseks tee XOR reg-ga D0
				goto	Faasup11_0				; D0 on 0 -> v�ljund = 1
;				btfsc	RegX+.0,.3				; v�ljund l�heb k�rgeks, kas juba on ?
;				goto	Faasup12				; jah, v�ta j�rgmine v�ljund
				bcf		Dout3				
				bcf		INDF0,.3				; kajasta ka v�ljundite seisu r/o registris	151
				goto	Faasup11ex				; tehtud !
Faasup11_0:		bsf		Dout3				
				bsf		INDF0,.3				; kajasta ka v�ljundite seisu r/o registris	151
Faasup11ex:		movff	pwm11set+.1,pwm11work+.1; loe pulsi kestus (tarbetu ?)
				movff	pwm11set+.0,pwm11work+.0
				bsf		RegX+.0,.3
Faasup12:		LFSR	.1,pwm12work+.0
				call	chkzero					; kas kestus on 0 ?
				btfsc	Seero
				goto	Faasup13				; jah, v�ta j�rgmine v�ljund
				btfss	INDF1,.6				; l�litame vaid sobival faasil ?
				goto	Faasup12nf				; eip, v�ib kohe l�litada
				swapf	INDF1,W					; jah, kas on �ige faas ?
				andlw	0x03
				subwf	phasecounter,W		
				btfss	ZERO
				goto	Faasup13				; ei, v�ta j�rgmine v�ljund
Faasup12nf:				btfsc	RegX+.0,.4				; v�ljund l�heb k�rgeks, kas juba on ?
				goto	Faasup13				; jah, v�ta j�rgmine v�ljund
		btfss	Register0+.0,.4			; v�ljundi m��ramiseks tee XOR reg-ga D0
				goto	Faasup12_0				; D0 on 0 -> v�ljund = 1
;				btfsc	RegX+.0,.4				; v�ljund l�heb k�rgeks, kas juba on ?
;				goto	Faasup13				; jah, v�ta j�rgmine v�ljund
				bcf		Dout4				
				bcf		INDF0,.4				; kajasta ka v�ljundite seisu r/o registris	151
				goto	Faasup12ex				; tehtud !
Faasup12_0:		bsf		Dout4				
				bsf		INDF0,.4				; kajasta ka v�ljundite seisu r/o registris	151
Faasup12ex:		movff	pwm12set+.1,pwm12work+.1; loe pulsi kestus (tarbetu ?)
				movff	pwm12set+.0,pwm12work+.0
				bsf		RegX+.0,.4
Faasup13:		LFSR	.1,pwm13work+.0
				call	chkzero					; kas kestus on 0 ?
				btfsc	Seero
				goto	Faasup14				; jah, v�ta j�rgmine v�ljund
				btfss	INDF1,.6				; l�litame vaid sobival faasil ?
				goto	Faasup13nf				; eip, v�ib kohe l�litada
				swapf	INDF1,W					; jah, kas on �ige faas ?
				andlw	0x03
				subwf	phasecounter,W		
				btfss	ZERO
				goto	Faasup14				; ei, v�ta j�rgmine v�ljund
Faasup13nf:				btfsc	RegX+.0,.5				; v�ljund l�heb k�rgeks, kas juba on ?
				goto	Faasup14				; jah, v�ta j�rgmine v�ljund
		btfss	Register0+.0,.5			; v�ljundi m��ramiseks tee XOR reg-ga D0
				goto	Faasup13_0				; D0 on 0 -> v�ljund = 1
;				btfsc	RegX+.0,.5				; v�ljund l�heb k�rgeks, kas juba on ?
;				goto	Faasup14				; jah, v�ta j�rgmine v�ljund
				bcf		Dout5				
				bcf		INDF0,.5				; kajasta ka v�ljundite seisu r/o registris	151
				goto	Faasup13ex				; tehtud !
Faasup13_0:		bsf		Dout5				
				bsf		INDF0,.5				; kajasta ka v�ljundite seisu r/o registris	151
Faasup13ex:		movff	pwm13set+.1,pwm13work+.1; loe pulsi kestus (tarbetu ?)
				movff	pwm13set+.0,pwm13work+.0
				bsf		RegX+.0,.5
Faasup14:		LFSR	.1,pwm14work+.0
				call	chkzero					; kas kestus on 0 ?
				btfsc	Seero
				goto	Faasup15				; jah, v�ta j�rgmine v�ljund
				btfss	INDF1,.6				; l�litame vaid sobival faasil ?
				goto	Faasup14nf				; eip, v�ib kohe l�litada
				swapf	INDF1,W					; jah, kas on �ige faas ?
				andlw	0x03
				subwf	phasecounter,W		
				btfss	ZERO
				goto	Faasup15				; ei, v�ta j�rgmine v�ljund
Faasup14nf:					btfsc	RegX+.0,.6				; v�ljund l�heb k�rgeks, kas juba on ?
				goto	Faasup15				; jah, v�ta j�rgmine v�ljund
	btfss	Register0+.0,.6			; v�ljundi m��ramiseks tee XOR reg-ga D0
				goto	Faasup14_0				; D0 on 0 -> v�ljund = 1
;				btfsc	RegX+.0,.6				; v�ljund l�heb k�rgeks, kas juba on ?
;				goto	Faasup15				; jah, v�ta j�rgmine v�ljund
				bcf		Dout6				
				bcf		INDF0,.6				; kajasta ka v�ljundite seisu r/o registris	151
				goto	Faasup14ex				; tehtud !
Faasup14_0:		bsf		Dout6				
				bsf		INDF0,.6				; kajasta ka v�ljundite seisu r/o registris	151
Faasup14ex:		movff	pwm14set+.1,pwm14work+.1; loe pulsi kestus (tarbetu ?)
				movff	pwm14set+.0,pwm14work+.0
				bsf		RegX+.0,.6
Faasup15:		LFSR	.1,pwm15work+.0
				call	chkzero					; kas kestus on 0 ?
				btfsc	Seero
				goto	Faasup16				; jah, v�ta j�rgmine v�ljund
				btfss	INDF1,.6				; l�litame vaid sobival faasil ?
				goto	Faasup15nf				; eip, v�ib kohe l�litada
				swapf	INDF1,W					; jah, kas on �ige faas ?
				andlw	0x03
				subwf	phasecounter,W		
				btfss	ZERO
				goto	Faasup16				; ei, v�ta j�rgmine v�ljund
Faasup15nf:				btfsc	RegX+.0,.7				; v�ljund l�heb k�rgeks, kas juba on ?
				goto	Faasup16				; jah, v�ta j�rgmine v�ljund
		btfss	Register0+.0,.7			; v�ljundi m��ramiseks tee XOR reg-ga D0
				goto	Faasup15_0				; D0 on 0 -> v�ljund = 1
;				btfsc	RegX+.0,.7				; v�ljund l�heb k�rgeks, kas juba on ?
;				goto	Faasup16				; jah, v�ta j�rgmine v�ljund
				bcf		Dout7				
				bcf		INDF0,.7				; kajasta ka v�ljundite seisu r/o registris	151
				goto	Faasup15ex				; tehtud !
Faasup15_0:		bsf		Dout7				
				bsf		INDF0,.7				; kajasta ka v�ljundite seisu r/o registris	151
Faasup15ex:		movff	pwm15set+.1,pwm15work+.1; loe pulsi kestus (tarbetu ?)
				movff	pwm15set+.0,pwm15work+.0
				bsf		RegX+.0,.7
Faasup16:		goto	T0intPulEnd				; j��b veel �ige nati teha... :)
decpertick:		movff	periodtick+.0,pwmtmp+.0
				movff	periodtick+.1,pwmtmp+.1
				movff	periodtick+.2,pwmtmp+.2
				bcf		CARRY
				decf	pwmtmp+.2,F
				btfsc	CARRY
				goto	decpertick1
				bcf		CARRY
				decf	pwmtmp+.1,F
				btfsc	CARRY
				goto	decpertick1
				bcf		CARRY
				decf	pwmtmp+.0,F
decpertick1:	movff	pwmtmp+.0,periodtick+.0
				movff	pwmtmp+.1,periodtick+.1
				movff	pwmtmp+.2,periodtick+.2
				movff	periodtick+.0,WREG		; periood sai n��d l�bi ?
				addlw	.0
				btfss	ZERO
				goto	decpertick1a			; eip
				movff	periodtick+.1,WREG
				addlw	.0
				btfss	ZERO
				goto	decpertick1a
				movff	periodtick+.2,WREG
				addlw	.0
				btfss	ZERO
				goto	decpertick1a
				goto	T0intfaas				; sai l�bi, alustame uut

decpertick1a:	movff	phasetick+.0,pwmtmp+.0	; faasiloendist ka 1 samm maha
				movff	phasetick+.1,pwmtmp+.1
				bcf		CARRY
				decf	pwmtmp+.1,F
				btfsc	CARRY
				goto	decpertick2
				bcf		CARRY
				decf	pwmtmp+.0,F
decpertick2:	movff	pwmtmp+.0,phasetick+.0
				movff	pwmtmp+.1,phasetick+.1
				movff	phasetick+.1,WREG		; faasi kestus otcas ?
				addlw	.0
				btfss	ZERO
				goto	T0intPulEnd				; eip
				movff	phasetick+.0,WREG		
				addlw	.0
				btfss	ZERO
				goto	T0intPulEnd				; eip
				movff	mphasetick+.0,phasetick+.0; jah, taasta faasi kestus
				movff	mphasetick+.1,phasetick+.1			
				movff	phasecounter,WREG		; suurendame perioodide (faaside) loendit
				addlw	.1
				movff	WREG,phasecounter
				movff	phasecounter,WREG		
				sublw	.4						; on max 4 faasi, �le selle p��rab algusesse tagasi
				btfss	ZERO
				goto	OutHigh
				movlw	0x00	
				movff	WREG,phasecounter
				goto	OutHigh

; ***** pulsi l�petamine *****
T0intPulEnd:	decfsz	_1mscount				; 1 ms l�bi ?
				goto	T0intend				; eip, siis ei tee midagi
				movlw	.4						; jaap, taasta 1ms loendi
				movwf	_1mscount
				LFSR	.0,Register151+.1		; kas antud v�ljundi pulss vaja l�petada ?
FaasDn0:		LFSR	.1,pwm0work				; viita j�rgmisele v�ljundile
				call	chkzero					; jah, kas juba null (nt ei peagi pulssi olema) ?
				btfsc	Seero
				goto	FaasDn0Z				; ongi null ! L�� v�ljund ka nulli.
				btfss	RegX+.1,.0				; on pulss �ldse alanud ?
				goto	FaasDn1					; eip, v�ta j�rgmine v�ljund
				call	decrcount				; ei ole veel 0, tixu kestust v�hemaks
				call	chkzero					; kas n��d on 0 ?
				btfss	Seero
				goto	FaasDn1					; eip, v�ta j�rgmine v�ljund
FaasDn0Z:		btfss	Register0+.1,.0			; reaalse v�ljundi m��ramiseks tee XOR reg-ga D0
				goto	FaasDn0_0				; D0 on 0 -> v�ljund = 0
				bsf		PORTA,.0				; D0 on 1 -> v�ljund = 1 
				bsf		INDF0,.0				; kajasta ka v�ljundite seisu r/o registris	151
				goto	Faasdn0ex				; tehtud !
FaasDn0_0:		bcf		PORTA,.0				; D0 on 0 -> v�ljund = 0 
				bcf		INDF0,.0				; kajasta ka v�ljundite seisu r/o registris	151
Faasdn0ex:		bcf		RegX+.1,.0				; m�rgi �ra pwm'i (positiivse) loogika registrisse
				movff	pwm0set+.0,pwm0work+.0	; taasta pulsi kestus
				movff	pwm0set+.1,pwm0work+.1
				btfsc	INDF1,.7				; kui �hekordne pulss siis teeme edaspidi kestuse nullix
				goto	FaasDn1					; korduv - ei n�pi rohkem !
				movff	pwm0work+.0,WREG		; �hekordne ja see sai n��d tekitatud
				andlw	0xF0					; seega j�rgnevate pulsside kestus nullix
				movff	WREG,pwm0work+.0
				movlw	0x00
				movff	WREG,pwm0work+.1
FaasDn1:		LFSR	.1,pwm1work				; viita j�rgmisele v�ljundile
				call	chkzero					; jah, kas juba null (nt ei peagi pulssi olema) ?
				btfsc	Seero
				goto	FaasDn1Z				; ongi null ! L�� v�ljund ka nulli.
				btfss	RegX+.1,.1				; on pulss �ldse alanud ?
				goto	FaasDn2					; eip, v�ta j�rgmine v�ljund
				call	decrcount				; ei ole veel 0, tixu kestust v�hemaks
				call	chkzero					; kas n��d on 0 ?
				btfss	Seero
				goto	FaasDn2					; eip, v�ta j�rgmine v�ljund
FaasDn1Z:		btfss	Register0+.1,.1			; reaalse v�ljundi m��ramiseks tee XOR reg-ga D0
				goto	FaasDn1_0				; D0 on 0 -> v�ljund = 0
				bsf		PORTA,.1				; D0 on 1 -> v�ljund = 1 
				bsf		INDF0,.1				; kajasta ka v�ljundite seisu r/o registris	151
				goto	Faasdn1ex				; tehtud !
FaasDn1_0:		bcf		PORTA,.1				; D0 on 0 -> v�ljund = 0 
				bcf		INDF0,.1				; kajasta ka v�ljundite seisu r/o registris	151
Faasdn1ex:		bcf		RegX+.1,.1				; m�rgi �ra pwm'i (positiivse) loogika registrisse
				movff	pwm1set+.0,pwm1work+.0	; taasta pulsi kestus
				movff	pwm1set+.1,pwm1work+.1
				btfsc	INDF1,.7				; kui �hekordne pulss siis teeme edaspidi kestuse nullix
				goto	FaasDn2					; korduv - ei n�pi rohkem !
				movff	pwm1work+.0,WREG		; �hekordne ja see sai n��d tekitatud
				andlw	0xF0					; seega j�rgnevate pulsside kestus nullix
				movff	WREG,pwm1work+.0
				movlw	0x00
				movff	WREG,pwm1work+.1
FaasDn2:		LFSR	.1,pwm2work				; viita j�rgmisele v�ljundile
				call	chkzero					; jah, kas juba null (nt ei peagi pulssi olema) ?
				btfsc	Seero
				goto	FaasDn2Z				; ongi null ! L�� v�ljund ka nulli.
				btfss	RegX+.1,.2				; on pulss �ldse alanud ?
				goto	FaasDn3					; eip, v�ta j�rgmine v�ljund
				call	decrcount				; ei ole veel 0, tixu kestust v�hemaks
				call	chkzero					; kas n��d on 0 ?
				btfss	Seero
				goto	FaasDn3					; eip, v�ta j�rgmine v�ljund
FaasDn2Z:		btfss	Register0+.1,.2			; reaalse v�ljundi m��ramiseks tee XOR reg-ga D0
				goto	FaasDn2_0				; D0 on 0 -> v�ljund = 0
				bsf		PORTA,.2				; D0 on 1 -> v�ljund = 1 
				bsf		INDF0,.2				; kajasta ka v�ljundite seisu r/o registris	151
				goto	Faasdn2ex				; tehtud !
FaasDn2_0:		bcf		PORTA,.2				; D0 on 0 -> v�ljund = 0 
				bcf		INDF0,.2				; kajasta ka v�ljundite seisu r/o registris	151
Faasdn2ex:		bcf		RegX+.1,.2				; m�rgi �ra pwm'i (positiivse) loogika registrisse
				movff	pwm2set+.0,pwm2work+.0	; taasta pulsi kestus
				movff	pwm2set+.1,pwm2work+.1
				btfsc	INDF1,.7				; kui �hekordne pulss siis teeme edaspidi kestuse nullix
				goto	FaasDn3					; korduv - ei n�pi rohkem !
				movff	pwm2work+.0,WREG		; �hekordne ja see sai n��d tekitatud
				andlw	0xF0					; seega j�rgnevate pulsside kestus nullix
				movff	WREG,pwm2work+.0
				movlw	0x00
				movff	WREG,pwm2work+.1
FaasDn3:		LFSR	.1,pwm3work				; viita j�rgmisele v�ljundile
				call	chkzero					; jah, kas juba null (nt ei peagi pulssi olema) ?
				btfsc	Seero
				goto	FaasDn3Z				; ongi null ! L�� v�ljund ka nulli.
				btfss	RegX+.1,.3				; on pulss �ldse alanud ?
				goto	FaasDn4					; eip, v�ta j�rgmine v�ljund
				call	decrcount				; ei ole veel 0, tixu kestust v�hemaks
				call	chkzero					; kas n��d on 0 ?
				btfss	Seero
				goto	FaasDn4					; eip, v�ta j�rgmine v�ljund
FaasDn3Z:		btfss	Register0+.1,.3			; reaalse v�ljundi m��ramiseks tee XOR reg-ga D0
				goto	FaasDn3_0				; D0 on 0 -> v�ljund = 0
				bsf		PORTA,.3				; D0 on 1 -> v�ljund = 1 
				bsf		INDF0,.3				; kajasta ka v�ljundite seisu r/o registris	151
				goto	Faasdn3ex				; tehtud !
FaasDn3_0:		bcf		PORTA,.3				; D0 on 0 -> v�ljund = 0 
				bcf		INDF0,.3				; kajasta ka v�ljundite seisu r/o registris	151
Faasdn3ex:		bcf		RegX+.1,.3				; m�rgi �ra pwm'i (positiivse) loogika registrisse
				movff	pwm3set+.0,pwm3work+.0	; taasta pulsi kestus
				movff	pwm3set+.1,pwm3work+.1
				btfsc	INDF1,.7				; kui �hekordne pulss siis teeme edaspidi kestuse nullix
				goto	FaasDn4					; korduv - ei n�pi rohkem !
				movff	pwm3work+.0,WREG		; �hekordne ja see sai n��d tekitatud
				andlw	0xF0					; seega j�rgnevate pulsside kestus nullix
				movff	WREG,pwm3work+.0
				movlw	0x00
				movff	WREG,pwm3work+.1
FaasDn4:		LFSR	.1,pwm4work				; viita j�rgmisele v�ljundile
				call	chkzero					; jah, kas juba null (nt ei peagi pulssi olema) ?
				btfsc	Seero
				goto	FaasDn4Z				; ongi null ! L�� v�ljund ka nulli.
				btfss	RegX+.1,.4				; on pulss �ldse alanud ?
				goto	FaasDn5					; eip, v�ta j�rgmine v�ljund
				call	decrcount				; ei ole veel 0, tixu kestust v�hemaks
				call	chkzero					; kas n��d on 0 ?
				btfss	Seero
				goto	FaasDn5					; eip, v�ta j�rgmine v�ljund
FaasDn4Z:		btfss	Register0+.1,.4			; reaalse v�ljundi m��ramiseks tee XOR reg-ga D0
				goto	FaasDn4_0				; D0 on 0 -> v�ljund = 0
				bsf		PORTA,.5				; D0 on 1 -> v�ljund = 1 
				bsf		INDF0,.4				; kajasta ka v�ljundite seisu r/o registris	151
				goto	Faasdn4ex				; tehtud !
FaasDn4_0:		bcf		PORTA,.5				; D0 on 0 -> v�ljund = 0 
				bcf		INDF0,.4				; kajasta ka v�ljundite seisu r/o registris	151
Faasdn4ex:		bcf		RegX+.1,.4				; m�rgi �ra pwm'i (positiivse) loogika registrisse
				movff	pwm4set+.0,pwm4work+.0	; taasta pulsi kestus
				movff	pwm4set+.1,pwm4work+.1
				btfsc	INDF1,.7				; kui �hekordne pulss siis teeme edaspidi kestuse nullix
				goto	FaasDn5					; korduv - ei n�pi rohkem !
				movff	pwm4work+.0,WREG		; �hekordne ja see sai n��d tekitatud
				andlw	0xF0					; seega j�rgnevate pulsside kestus nullix
				movff	WREG,pwm4work+.0
				movlw	0x00
				movff	WREG,pwm4work+.1
FaasDn5:		LFSR	.1,pwm5work				; viita j�rgmisele v�ljundile
				call	chkzero					; jah, kas juba null (nt ei peagi pulssi olema) ?
				btfsc	Seero
				goto	FaasDn5Z				; ongi null ! L�� v�ljund ka nulli.
				btfss	RegX+.1,.5				; on pulss �ldse alanud ?
				goto	FaasDn6					; eip, v�ta j�rgmine v�ljund
				call	decrcount				; ei ole veel 0, tixu kestust v�hemaks
				call	chkzero					; kas n��d on 0 ?
				btfss	Seero
				goto	FaasDn6					; eip, v�ta j�rgmine v�ljund
FaasDn5Z:		btfss	Register0+.1,.5			; reaalse v�ljundi m��ramiseks tee XOR reg-ga D0
				goto	FaasDn5_0				; D0 on 0 -> v�ljund = 0
				bsf		PORTE,.0				; D0 on 1 -> v�ljund = 1 
				bsf		INDF0,.5				; kajasta ka v�ljundite seisu r/o registris	151
				goto	Faasdn5ex				; tehtud !
FaasDn5_0:		bcf		PORTE,.0				; D0 on 0 -> v�ljund = 0 
				bcf		INDF0,.5				; kajasta ka v�ljundite seisu r/o registris	151
Faasdn5ex:		bcf		RegX+.1,.5				; m�rgi �ra pwm'i (positiivse) loogika registrisse
				movff	pwm5set+.0,pwm5work+.0	; taasta pulsi kestus
				movff	pwm5set+.1,pwm5work+.1
				btfsc	INDF1,.7				; kui �hekordne pulss siis teeme edaspidi kestuse nullix
				goto	FaasDn6					; korduv - ei n�pi rohkem !
				movff	pwm5work+.0,WREG		; �hekordne ja see sai n��d tekitatud
				andlw	0xF0					; seega j�rgnevate pulsside kestus nullix
				movff	WREG,pwm5work+.0
				movlw	0x00
				movff	WREG,pwm5work+.1
FaasDn6:		LFSR	.1,pwm6work				; viita j�rgmisele v�ljundile
				call	chkzero					; jah, kas juba null (nt ei peagi pulssi olema) ?
				btfsc	Seero
				goto	FaasDn6Z				; ongi null ! L�� v�ljund ka nulli.
				btfss	RegX+.1,.6				; on pulss �ldse alanud ?
				goto	FaasDn7					; eip, v�ta j�rgmine v�ljund
				call	decrcount				; ei ole veel 0, tixu kestust v�hemaks
				call	chkzero					; kas n��d on 0 ?
				btfss	Seero
				goto	FaasDn7					; eip, v�ta j�rgmine v�ljund
FaasDn6Z:		btfss	Register0+.1,.6			; reaalse v�ljundi m��ramiseks tee XOR reg-ga D0
				goto	FaasDn6_0				; D0 on 0 -> v�ljund = 0
				bsf		PORTE,.1				; D0 on 1 -> v�ljund = 1 
				bsf		INDF0,.6				; kajasta ka v�ljundite seisu r/o registris	151
				goto	Faasdn6ex				; tehtud !
FaasDn6_0:		bcf		PORTE,.1				; D0 on 0 -> v�ljund = 0 
				bcf		INDF0,.6				; kajasta ka v�ljundite seisu r/o registris	151
Faasdn6ex:		bcf		RegX+.1,.6				; m�rgi �ra pwm'i (positiivse) loogika registrisse
				movff	pwm6set+.0,pwm6work+.0	; taasta pulsi kestus
				movff	pwm6set+.1,pwm6work+.1
				btfsc	INDF1,.7				; kui �hekordne pulss siis teeme edaspidi kestuse nullix
				goto	FaasDn7					; korduv - ei n�pi rohkem !
				movff	pwm6work+.0,WREG		; �hekordne ja see sai n��d tekitatud
				andlw	0xF0					; seega j�rgnevate pulsside kestus nullix
				movff	WREG,pwm6work+.0
				movlw	0x00
				movff	WREG,pwm6work+.1
FaasDn7:		LFSR	.1,pwm7work				; viita j�rgmisele v�ljundile
				call	chkzero					; jah, kas juba null (nt ei peagi pulssi olema) ?
				btfsc	Seero
				goto	FaasDn7Z				; ongi null ! L�� v�ljund ka nulli.
				btfss	RegX+.1,.7				; on pulss �ldse alanud ?
				goto	FaasDn8					; eip, v�ta j�rgmine v�ljund
				call	decrcount				; ei ole veel 0, tixu kestust v�hemaks
				call	chkzero					; kas n��d on 0 ?
				btfss	Seero
				goto	FaasDn8					; eip, v�ta j�rgmine v�ljund
FaasDn7Z:		btfss	Register0+.1,.7			; reaalse v�ljundi m��ramiseks tee XOR reg-ga D0
				goto	FaasDn7_0				; D0 on 0 -> v�ljund = 0
				bsf		PORTE,.2				; D0 on 1 -> v�ljund = 1 
				bsf		INDF0,.7				; kajasta ka v�ljundite seisu r/o registris	151
				goto	Faasdn7ex				; tehtud !
FaasDn7_0:		bcf		PORTE,.2				; D0 on 0 -> v�ljund = 0 
				bcf		INDF0,.7				; kajasta ka v�ljundite seisu r/o registris	151
Faasdn7ex:		bcf		RegX+.1,.7				; m�rgi �ra pwm'i (positiivse) loogika registrisse
				movff	pwm7set+.0,pwm7work+.0	; taasta pulsi kestus
				movff	pwm7set+.1,pwm7work+.1
				btfsc	INDF1,.7				; kui �hekordne pulss siis teeme edaspidi kestuse nullix
				goto	FaasDn8					; korduv - ei n�pi rohkem !
				movff	pwm7work+.0,WREG		; �hekordne ja see sai n��d tekitatud
				andlw	0xF0					; seega j�rgnevate pulsside kestus nullix
				movff	WREG,pwm7work+.0
				movlw	0x00
				movff	WREG,pwm7work+.1
; **** Digipordi v�ljundid ****
FaasDn8:		LFSR	.1,pwm8work				; viita j�rgmisele v�ljundile
				LFSR	.0,Register151+.0		; siia l�heb kirja v�ljundi reaalne seis
				call	chkzero					; jah, kas juba null (nt ei peagi pulssi olema) ?
				btfsc	Seero
				goto	FaasDn8Z				; ongi null ! L�� v�ljund ka nulli.
				btfss	RegX+.0,.0				; on pulss �ldse alanud ?
				goto	FaasDn9					; eip, v�ta j�rgmine v�ljund
				call	decrcount				; ei ole veel 0, tixu kestust v�hemaks
				call	chkzero					; kas n��d on 0 ?
				btfss	Seero
				goto	FaasDn9					; eip, v�ta j�rgmine v�ljund
FaasDn8Z:		btfss	Register0+.0,.0			; reaalse v�ljundi m��ramiseks tee XOR reg-ga D0
				goto	FaasDn8_0				; D0 on 0 -> v�ljund = 0
				bsf		Dout0					; D0 on 1 -> v�ljund = 1 
				bsf		INDF0,.0				; kajasta ka v�ljundite seisu r/o registris	151
				goto	Faasdn8ex				; tehtud !
FaasDn8_0:		bcf		Dout0					; D0 on 0 -> v�ljund = 0 
				bcf		INDF0,.0				; kajasta ka v�ljundite seisu r/o registris	151
Faasdn8ex:		bcf		RegX+.0,.0				; m�rgi �ra pwm'i (positiivse) loogika registrisse
				movff	pwm8set+.0,pwm8work+.0	; taasta pulsi kestus
				movff	pwm8set+.1,pwm8work+.1
				btfsc	INDF1,.7				; kui �hekordne pulss siis teeme edaspidi kestuse nullix
				goto	FaasDn9					; korduv - ei n�pi rohkem !
				movff	pwm8work+.0,WREG		; �hekordne ja see sai n��d tekitatud
				andlw	0xF0					; seega j�rgnevate pulsside kestus nullix
				movff	WREG,pwm8work+.0
				movlw	0x00
				movff	WREG,pwm8work+.1
FaasDn9:		LFSR	.1,pwm9work				; viita j�rgmisele v�ljundile
				call	chkzero					; jah, kas juba null (nt ei peagi pulssi olema) ?
				btfsc	Seero
				goto	FaasDn9Z				; ongi null ! L�� v�ljund ka nulli.
				btfss	RegX+.0,.1				; on pulss �ldse alanud ?
				goto	FaasDn10				; eip, v�ta j�rgmine v�ljund
				call	decrcount				; ei ole veel 0, tixu kestust v�hemaks
				call	chkzero					; kas n��d on 0 ?
				btfss	Seero
				goto	FaasDn10				; eip, v�ta j�rgmine v�ljund
FaasDn9Z:		btfss	Register0+.0,.1			; reaalse v�ljundi m��ramiseks tee XOR reg-ga D0
				goto	FaasDn9_0				; D0 on 0 -> v�ljund = 0
				bsf		Dout1					; D0 on 1 -> v�ljund = 1 
				bsf		INDF0,.1				; kajasta ka v�ljundite seisu r/o registris	151
				goto	Faasdn9ex				; tehtud !
FaasDn9_0:		bcf		Dout1					; D0 on 0 -> v�ljund = 0 
				bcf		INDF0,.1				; kajasta ka v�ljundite seisu r/o registris	151
Faasdn9ex:		bcf		RegX+.0,.1				; m�rgi �ra pwm'i (positiivse) loogika registrisse
				movff	pwm9set+.0,pwm9work+.0	; taasta pulsi kestus
				movff	pwm9set+.1,pwm9work+.1
				btfsc	INDF1,.7				; kui �hekordne pulss siis teeme edaspidi kestuse nullix
				goto	FaasDn10				; korduv - ei n�pi rohkem !
				movff	pwm9work+.0,WREG		; �hekordne ja see sai n��d tekitatud
				andlw	0xF0					; seega j�rgnevate pulsside kestus nullix
				movff	WREG,pwm9work+.0
				movlw	0x00
				movff	WREG,pwm9work+.1
FaasDn10:		LFSR	.1,pwm10work			; viita j�rgmisele v�ljundile
				call	chkzero					; jah, kas juba null (nt ei peagi pulssi olema) ?
				btfsc	Seero
				goto	FaasDn10Z				; ongi null ! L�� v�ljund ka nulli.
				btfss	RegX+.0,.2				; on pulss �ldse alanud ?
				goto	FaasDn11				; eip, v�ta j�rgmine v�ljund
				call	decrcount				; ei ole veel 0, tixu kestust v�hemaks
				call	chkzero					; kas n��d on 0 ?
				btfss	Seero
				goto	FaasDn11				; eip, v�ta j�rgmine v�ljund
FaasDn10Z:		btfss	Register0+.0,.2			; reaalse v�ljundi m��ramiseks tee XOR reg-ga D0
				goto	FaasDn10_0				; D0 on 0 -> v�ljund = 0
				bsf		Dout2					; D0 on 1 -> v�ljund = 1 
				bsf		INDF0,.2				; kajasta ka v�ljundite seisu r/o registris	151
				goto	Faasdn10ex				; tehtud !
FaasDn10_0:		bcf		Dout2					; D0 on 0 -> v�ljund = 0 
				bcf		INDF0,.2				; kajasta ka v�ljundite seisu r/o registris	151
Faasdn10ex:		bcf		RegX+.0,.2				; m�rgi �ra pwm'i (positiivse) loogika registrisse
				movff	pwm10set+.0,pwm10work+.0; taasta pulsi kestus
				movff	pwm10set+.1,pwm10work+.1
				btfsc	INDF1,.7				; kui �hekordne pulss siis teeme edaspidi kestuse nullix
				goto	FaasDn11				; korduv - ei n�pi rohkem !
				movff	pwm10work+.0,WREG		; �hekordne ja see sai n��d tekitatud
				andlw	0xF0					; seega j�rgnevate pulsside kestus nullix
				movff	WREG,pwm10work+.0
				movlw	0x00
				movff	WREG,pwm10work+.1
FaasDn11:		LFSR	.1,pwm11work			; viita j�rgmisele v�ljundile
				call	chkzero					; jah, kas juba null (nt ei peagi pulssi olema) ?
				btfsc	Seero
				goto	FaasDn11Z				; ongi null ! L�� v�ljund ka nulli.
				btfss	RegX+.0,.3				; on pulss �ldse alanud ?
				goto	FaasDn12				; eip, v�ta j�rgmine v�ljund
				call	decrcount				; ei ole veel 0, tixu kestust v�hemaks
				call	chkzero					; kas n��d on 0 ?
				btfss	Seero
				goto	FaasDn12				; eip, v�ta j�rgmine v�ljund
FaasDn11Z:		btfss	Register0+.0,.3			; reaalse v�ljundi m��ramiseks tee XOR reg-ga D0
				goto	FaasDn11_0				; D0 on 0 -> v�ljund = 0
				bsf		Dout3					; D0 on 1 -> v�ljund = 1 
				bsf		INDF0,.3				; kajasta ka v�ljundite seisu r/o registris	151
				goto	Faasdn11ex				; tehtud !
FaasDn11_0:		bcf		Dout3					; D0 on 0 -> v�ljund = 0 
				bcf		INDF0,.3				; kajasta ka v�ljundite seisu r/o registris	151
Faasdn11ex:		bcf		RegX+.0,.3				; m�rgi �ra pwm'i (positiivse) loogika registrisse
				movff	pwm11set+.0,pwm11work+.0; taasta pulsi kestus
				movff	pwm11set+.1,pwm11work+.1
				btfsc	INDF1,.7				; kui �hekordne pulss siis teeme edaspidi kestuse nullix
				goto	FaasDn12				; korduv - ei n�pi rohkem !
				movff	pwm11work+.0,WREG		; �hekordne ja see sai n��d tekitatud
				andlw	0xF0					; seega j�rgnevate pulsside kestus nullix
				movff	WREG,pwm11work+.0
				movlw	0x00
				movff	WREG,pwm11work+.1
FaasDn12:		LFSR	.1,pwm12work			; viita j�rgmisele v�ljundile
				call	chkzero					; jah, kas juba null (nt ei peagi pulssi olema) ?
				btfsc	Seero
				goto	FaasDn12Z				; ongi null ! L�� v�ljund ka nulli.
				btfss	RegX+.0,.4				; on pulss �ldse alanud ?
				goto	FaasDn13				; eip, v�ta j�rgmine v�ljund
				call	decrcount				; ei ole veel 0, tixu kestust v�hemaks
				call	chkzero					; kas n��d on 0 ?
				btfss	Seero
				goto	FaasDn13				; eip, v�ta j�rgmine v�ljund
FaasDn12Z:		btfss	Register0+.0,.4			; reaalse v�ljundi m��ramiseks tee XOR reg-ga D0
				goto	FaasDn12_0				; D0 on 0 -> v�ljund = 0
				bsf		Dout4					; D0 on 1 -> v�ljund = 1 
				bsf		INDF0,.4				; kajasta ka v�ljundite seisu r/o registris	151
				goto	Faasdn12ex				; tehtud !
FaasDn12_0:		bcf		Dout4					; D0 on 0 -> v�ljund = 0 
				bcf		INDF0,.4				; kajasta ka v�ljundite seisu r/o registris	151
Faasdn12ex:		bcf		RegX+.0,.4				; m�rgi �ra pwm'i (positiivse) loogika registrisse
				movff	pwm12set+.0,pwm12work+.0; taasta pulsi kestus
				movff	pwm12set+.1,pwm12work+.1
				btfsc	INDF1,.7				; kui �hekordne pulss siis teeme edaspidi kestuse nullix
				goto	FaasDn13				; korduv - ei n�pi rohkem !
				movff	pwm12work+.0,WREG		; �hekordne ja see sai n��d tekitatud
				andlw	0xF0					; seega j�rgnevate pulsside kestus nullix
				movff	WREG,pwm12work+.0
				movlw	0x00
				movff	WREG,pwm12work+.1
FaasDn13:		LFSR	.1,pwm13work			; viita j�rgmisele v�ljundile
				call	chkzero					; jah, kas juba null (nt ei peagi pulssi olema) ?
				btfsc	Seero
				goto	FaasDn13Z				; ongi null ! L�� v�ljund ka nulli.
				btfss	RegX+.0,.5				; on pulss �ldse alanud ?
				goto	FaasDn14				; eip, v�ta j�rgmine v�ljund
				call	decrcount				; ei ole veel 0, tixu kestust v�hemaks
				call	chkzero					; kas n��d on 0 ?
				btfss	Seero
				goto	FaasDn14				; eip, v�ta j�rgmine v�ljund
FaasDn13Z:		btfss	Register0+.0,.5			; reaalse v�ljundi m��ramiseks tee XOR reg-ga D0
				goto	FaasDn13_0				; D0 on 0 -> v�ljund = 0
				bsf		Dout5					; D0 on 1 -> v�ljund = 1 
				bsf		INDF0,.5				; kajasta ka v�ljundite seisu r/o registris	151
				goto	Faasdn13ex				; tehtud !
FaasDn13_0:		bcf		Dout5					; D0 on 0 -> v�ljund = 0 
				bcf		INDF0,.5				; kajasta ka v�ljundite seisu r/o registris	151
Faasdn13ex:		bcf		RegX+.0,.5				; m�rgi �ra pwm'i (positiivse) loogika registrisse
				movff	pwm13set+.0,pwm13work+.0; taasta pulsi kestus
				movff	pwm13set+.1,pwm13work+.1
				btfsc	INDF1,.7				; kui �hekordne pulss siis teeme edaspidi kestuse nullix
				goto	FaasDn14				; korduv - ei n�pi rohkem !
				movff	pwm13work+.0,WREG		; �hekordne ja see sai n��d tekitatud
				andlw	0xF0					; seega j�rgnevate pulsside kestus nullix
				movff	WREG,pwm13work+.0
				movlw	0x00
				movff	WREG,pwm13work+.1
FaasDn14:		LFSR	.1,pwm14work			; viita j�rgmisele v�ljundile
				call	chkzero					; jah, kas juba null (nt ei peagi pulssi olema) ?
				btfsc	Seero
				goto	FaasDn14Z				; ongi null ! L�� v�ljund ka nulli.
				btfss	RegX+.0,.6				; on pulss �ldse alanud ?
				goto	FaasDn15				; eip, v�ta j�rgmine v�ljund
				call	decrcount				; ei ole veel 0, tixu kestust v�hemaks
				call	chkzero					; kas n��d on 0 ?
				btfss	Seero
				goto	FaasDn15				; eip, v�ta j�rgmine v�ljund
FaasDn14Z:		btfss	Register0+.0,.6			; reaalse v�ljundi m��ramiseks tee XOR reg-ga D0
				goto	FaasDn14_0				; D0 on 0 -> v�ljund = 0
				bsf		Dout6					; D0 on 1 -> v�ljund = 1 
				bsf		INDF0,.6				; kajasta ka v�ljundite seisu r/o registris	151
				goto	Faasdn14ex				; tehtud !
FaasDn14_0:		bcf		Dout6					; D0 on 0 -> v�ljund = 0 
				bcf		INDF0,.6				; kajasta ka v�ljundite seisu r/o registris	151
Faasdn14ex:		bcf		RegX+.0,.6				; m�rgi �ra pwm'i (positiivse) loogika registrisse
				movff	pwm14set+.0,pwm14work+.0; taasta pulsi kestus
				movff	pwm14set+.1,pwm14work+.1
				btfsc	INDF1,.7				; kui �hekordne pulss siis teeme edaspidi kestuse nullix
				goto	FaasDn15				; korduv - ei n�pi rohkem !
				movff	pwm14work+.0,WREG		; �hekordne ja see sai n��d tekitatud
				andlw	0xF0					; seega j�rgnevate pulsside kestus nullix
				movff	WREG,pwm14work+.0
				movlw	0x00
				movff	WREG,pwm14work+.1
FaasDn15:		LFSR	.1,pwm15work			; viita j�rgmisele v�ljundile
				call	chkzero					; jah, kas juba null (nt ei peagi pulssi olema) ?
				btfsc	Seero
				goto	FaasDn15Z				; ongi null ! L�� v�ljund ka nulli.
				btfss	RegX+.0,.7				; on pulss �ldse alanud ?
				goto	FaasDn16				; eip, v�ta j�rgmine v�ljund
				call	decrcount				; ei ole veel 0, tixu kestust v�hemaks
				call	chkzero					; kas n��d on 0 ?
				btfss	Seero
				goto	FaasDn16				; eip, v�ta j�rgmine v�ljund
FaasDn15Z:		btfss	Register0+.0,.7			; reaalse v�ljundi m��ramiseks tee XOR reg-ga D0
				goto	FaasDn15_0				; D0 on 0 -> v�ljund = 0
				bsf		Dout7					; D0 on 1 -> v�ljund = 1 
				bsf		INDF0,.7				; kajasta ka v�ljundite seisu r/o registris	151
				goto	Faasdn15ex				; tehtud !
FaasDn15_0:		bcf		Dout7					; D0 on 0 -> v�ljund = 0 
				bcf		INDF0,.7				; kajasta ka v�ljundite seisu r/o registris	151
Faasdn15ex:		bcf		RegX+.0,.7				; m�rgi �ra pwm'i (positiivse) loogika registrisse
				movff	pwm15set+.0,pwm15work+.0; taasta pulsi kestus
				movff	pwm15set+.1,pwm15work+.1
				btfsc	INDF1,.7				; kui �hekordne pulss siis teeme edaspidi kestuse nullix
				goto	FaasDn16				; korduv - ei n�pi rohkem !
				movff	pwm15work+.0,WREG		; �hekordne ja see sai n��d tekitatud
				andlw	0xF0					; seega j�rgnevate pulsside kestus nullix
				movff	WREG,pwm15work+.0
				movlw	0x00
				movff	WREG,pwm15work+.1
FaasDn16:		bcf		CARRY					; loendame aega perioodi algusest
				incf	Register149out+.1,F
				btfsc	CARRY
				incf	Register149out+.0,F					
T0intend:		movff	FSR1tmpH,FSR1H			; taasta FSR1
				movff	FSR1tmpL,FSR1L			
				movff	FSRtmpL1,FSR0L			; ja FSR0 samuti
				movff	FSRtmpH1,FSR0H
				return
;===============================================================================
decrcount:		movf	POSTINC1,W				; viita LSB-le
				bcf		CARRY
				decf	POSTDEC1,F				; tixutame pulsi pikkust v�hemax
				btfsc	CARRY
				return
				decf	POSTDEC1,F
				movf	POSTINC1,W				; pointer olgu alguses tagasi  !
				return
;===============================================================================
chkzero:		bcf		Seero		
				movf	POSTINC1,W				; register nullis ?
				andlw	0x0F
				addlw	.0
				btfss	ZERO
				goto	chkzero1				; eip
				movf	POSTINC1,W
				addlw	.0
				btfss	ZERO
				goto	chkzero2				; eip
				bsf		Seero					; on 0 !
chkzero2:		movf	POSTDEC1,W				; korrigeeri pointerit		
chkzero1:		movf	POSTDEC1,W				; korrigeeri pointerit		
				return
;===============================================================================
; ******* Loendite signaali muutuse INT. Reageerivad langevale frondile! *******
;===============================================================================
Loendid:		movf	PORTB,W
				movwf	dataport				; Wiegandi jaux
				xorwf	lastinputs,W			; mis muutus ?
				movwf	muutus
				btfsc	muutus,.0				; loendi 1 ?
				goto	counter1				; jah
				btfsc	muutus,.1				; loendi 2 ?
				goto	counter2				; jah
				btfsc	muutus,.2				; loendi 3 ?
				goto	counter3				; jah
				btfsc	muutus,.3				; loendi 4 ?
				goto	counter4				; jah
				btfsc	muutus,.4				; loendi 5 ?
				goto	counter5				; jah
				btfsc	muutus,.5				; loendi 6 ?
				goto	counter6				; jah
				btfsc	muutus,.6				; loendi 7 ?
				goto	counter7				; jah
;				btfsc	muutus,.7				; loendi 8 ?
				goto	counter8				; jah
;				goto	Andur_end1;Andur_end	
; *** Loendi 1 ***
counter1:		btfss	dataport,.0				; reageerime vaid langevale frondile
				goto	counter1_1
				bsf		lastinputs,.0			; oli t�usev front, updteerime sisendite eelmist seisu
				goto	Andur_end				; ja vaatame j�rgmisi sisendeid
counter1_1:		bcf		lastinputs,.0			; updteerime sisendite eelmist seisu
				btfss	sens1tmron				; JUBA DEBOUBCEME ?
				goto	counter1_2				; ei veel, hakkab pihta
				goto	Andur_end				; jah, vaatame j�rgmisi sisendeid
counter1_2:		bsf		sens1tmron 				; taimer k�ima
				goto	Andur_end				; ja k�ik !
; *** Loendi 2 ***
counter2:		btfss	dataport,.1	
				goto	counter2_1
				bsf		lastinputs,.1
				goto	Andur_end	
counter2_1:		bcf		lastinputs,.1
				btfss	sens2tmron	
				goto	counter2_2	
				goto	Andur_end	
counter2_2:		bsf		sens2tmron 	
				goto	Andur_end	
; *** Loendi 3 ***
counter3:		btfss	dataport,.2	
				goto	counter3_1
				bsf		lastinputs,.2
				goto	Andur_end	
counter3_1:		bcf		lastinputs,.2
				btfss	sens3tmron	
				goto	counter3_2	
				goto	Andur_end	
counter3_2:		bsf		sens3tmron 	
				goto	Andur_end	
; *** Loendi 4 ***
counter4:		btfss	dataport,.3	
				goto	counter4_1
				bsf		lastinputs,.3
				goto	Andur_end	
counter4_1:		bcf		lastinputs,.3
				btfss	sens4tmron	
				goto	counter4_2	
				goto	Andur_end	
counter4_2:		bsf		sens4tmron 	
				goto	Andur_end	
; *** Loendi 5 v�i Wiegand B biti 0 sisend ***
counter5:; b5 - wiegand B lubatud (PORTB,4 ja 5)
; b6 - wiegand A lubatud (PORTB,6 ja 7)
				btfsc	Register273+.1,.5
				goto	Wiegand_Blow			; oli pulss ja Wiegand lubatud => t��tleme Wiegandi j�rgi
; *** tavaline Loendi 5 ***
				btfss	dataport,.4	
				goto	counter5_1
				bsf		lastinputs,.4
				goto	Andur_end	
counter5_1:		bcf		lastinputs,.4
				btfss	sens5tmron	
				goto	counter5_2	
				goto	Andur_end	
counter5_2:		bsf		sens5tmron 	
				goto	Andur_end	
; *** Loendi 6 v�i Wiegand B biti 1 sisend ***
counter6:		btfsc	Register273+.1,.5
				goto	Wiegand_Bhigh			
; *** tavaline Loendi 6 ***
				btfss	dataport,.5	
				goto	counter6_1
				bsf		lastinputs,.5
				goto	Andur_end	
counter6_1:		bcf		lastinputs,.5
				btfss	sens6tmron	
				goto	counter6_2	
				goto	Andur_end	
counter6_2:		bsf		sens6tmron 	
				goto	Andur_end	
; *** Loendi 7 v�i Wiegand A biti 0 sisend ***
counter7:		btfsc	Register273+.1,.6
				goto	Wiegand_Alow				
; *** tavaline Loendi 7 ***
				btfss	dataport,.6	
				goto	counter7_1
				bsf		lastinputs,.6
				goto	Andur_end	
counter7_1:		bcf		lastinputs,.6
				btfss	sens7tmron	
				goto	counter7_2	
				goto	Andur_end	
counter7_2:		bsf		sens7tmron 	
				goto	Andur_end	
; *** Loendi 7 v�i Wiegand A biti 1 sisend ***
counter8:		btfsc	Register273+.1,.6
				goto	Wiegand_Ahigh				
; *** tavaline Loendi 8 ***
				btfss	dataport,.7	
				goto	counter8_1
				bsf		lastinputs,.7
				goto	Andur_end	
counter8_1:		bcf		lastinputs,.7
				btfss	sens8tmron	
				goto	counter8_2	
				goto	Andur_end	
counter8_2:		bsf		sens8tmron 	
;				goto	Andur_end	
;===============================================================================
Andur_end:		movf	dataport,W				; rohkem muutusi ei ole (korraga rakendunud loendid) ?
				xorwf	lastinputs,W		
				btfss	ZERO
				return
Andur_end1:		bcf		INTCON,RBIF				; ei, siis v�tame katkestuse n�ude ka maha. Vastasel juhul tulex j�rgmisel korral tagasi ja analuusiks veel 
				return
;===============================================================================
; ******* PORTB sisendite (Wiegand) INT *******
;===============================================================================
Wiegand_Ahigh:
				banksel	PIE1
				bcf		PIE1,RC1IE
				banksel	.0
				btfsc	dataport,.7				; loeme vaid - fronti
				goto	wieg_endAH				; oli + front
				bcf		lastinputs,.7			; updateeri sisendite seisu
				btfsc	wiegandAto				; kui timeout siis on pakett juba k�es ja loeme edasi vaid siis kui master on data �ra lugenud
				goto	Andur_end				; minema siit !
				call	wieg_prepA				; + front, k�ivita paketi l�pu taimer
				call	rotH					; nihuta biti 1 v��rtus registrisse
				incf	WAbitcount,F			; loendame bitte
				goto	Andur_end				; aitab !
Wiegand_Alow:
				banksel	PIE1
				bcf		PIE1,RC1IE
				banksel	.0
				btfsc	dataport,.6				; loeme vaid - fronti
				goto	wieg_endAL				; oli + front
				bcf		lastinputs,.6;7			; updateeri sisendite seisu
				btfsc	wiegandAto				; kui timeout siis on pakett juba k�es ja loeme edasi vaid siis kui master on data �ra lugenud
				goto	Andur_end				; minema siit !
				call	wieg_prepA				; + front, k�ivita paketi l�pu taimer
				call	rotL					; nihuta biti 0 v��rtus registrisse
				incf	WAbitcount,F
				goto	Andur_end				; aitab !
Wiegand_Bhigh:
				banksel	PIE1
				bcf		PIE1,RC1IE
				banksel	.0
				btfsc	dataport,.5				; loeme vaid - fronti
				goto	wieg_endBH				; oli + front
				bcf		lastinputs,.5			; updateeri sisendite seisu
				btfsc	wiegandBto				; kui timeout siis on pakett juba k�es ja loeme edasi vaid siis kui master on data �ra lugenud
				goto	Andur_end				; minema siit !
				call	wieg_prepB				; + front, k�ivita paketi l�pu taimer
				call	rotH					; nihuta biti 1 v��rtus registrisse
				incf	WBbitcount,F
				goto	Andur_end				; aitab !

Wiegand_Blow:
				banksel	PIE1
				bcf		PIE1,RC1IE
				banksel	.0
				btfsc	dataport,.4				; loeme vaid - fronti
				goto	wieg_endBL				; oli + front
				bcf		lastinputs,.4			; updateeri sisendite seisu
				btfsc	wiegandBto				; kui timeout siis on pakett juba k�es ja loeme edasi vaid siis kui master on data �ra lugenud
				goto	Andur_end				; minema siit !
				call	wieg_prepB				; + front, k�ivita paketi l�pu taimer
				call	rotL					; nihuta biti 1 v��rtus registrisse
				incf	WBbitcount,F
				goto	Andur_end				; aitab !
;===============================================================================
wieg_prepA:		movlw	wiegandtime
				movwf	wiegandAtimer
				bsf		wiegandAtmron			; taimer paketi kestust valvama
				LFSR	.0,Register11+.7			; jooxva biti salvestamiseks vajaliku registri aadress
				return
wieg_prepB:		movlw	wiegandtime
				movwf	wiegandBtimer
				bsf		wiegandBtmron			; taimer paketi kestust valvama
				LFSR	.0,Register15+.7			; jooxva biti salvestamiseks vajaliku registri aadress
				return
;===============================================================================
rotH:;			bsf		STATUS,IRP				; 3. m�luplokk !
				bsf		CARRY
				goto	rot
rotL:;			bsf		STATUS,IRP
				bcf		CARRY
rot:			rlcf		INDF0,F
				decf	FSR0L,F
				rlcf		INDF0,F
				decf	FSR0L,F
				rlcf		INDF0,F
				decf	FSR0L,F
				rlcf		INDF0,F
				decf	FSR0L,F
				rlcf		INDF0,F
				decf	FSR0L,F
				rlcf		INDF0,F
				decf	FSR0L,F
				rlcf		INDF0,F
				decf	FSR0L,F
				rlcf		INDF0,F
				decf	FSR0L,F

;				bsf		STATUS,IRP				; tagasi maa peale...
				return
;===============================================================================
wieg_endAH:		bsf		lastinputs,.7			; updateeri sisendite seisu
				goto	Andur_end	
wieg_endAL:		bsf		lastinputs,.6			; updateeri sisendite seisu
				goto	Andur_end	
wieg_endBH:		bsf		lastinputs,.5			; updateeri sisendite seisu
				goto	Andur_end	
wieg_endBL:		bsf		lastinputs,.4			; updateeri sisendite seisu
				goto	Andur_end	
;===============================================================================
; ******* Seriali INT *******
;===============================================================================
SerInt:	
;********* Vahipeni ketti **************
				clrwdt                    		; WDT nullida 
				bcf		PIR1,RC1IF				; katkestuse n�ue maha
				banksel	RCSTA1
				movf	RCSTA1,W				; oli viga?
				banksel	.0
				andlw	.6						; Viga vastuv�tul? Maskeeri tarbetud staatuse bitid
				btfss	ZERO
				goto	reset_ser2				; oli, alusta uuesti
modbus_rcv:		movf	bytecnt,W				; kontrolli puhvri piire
				sublw	RSBufLen-.1
				btfss	CARRY
				goto	reset_ser2				; liiga pikk mess v�i miskit sassis, reset!
				bsf		SerialTimerOn	
				banksel	RCREG1
				movf	RCREG1,W
				banksel .0
				movwf	Char
				LFSR	.0,Puhver				; arvutame baidi salvestamise koha vastuv�tupuhvris
				movf	bytecnt,W
				bcf		CARRY
				addwf	FSR0L,F
				btfsc	CARRY
				incf	FSR0H,F
				movf	Char,W
				movwf	INDF0					; salvesta bait
				incf	bytecnt,F
				movf	bytecnt,W				; mitmes bait oli
				sublw	.1
				btfsc	ZERO
				goto	modb_r0					; esimene
				movf	bytecnt,W				; �kki oli teine (k�sk) ?
				sublw	.2
				btfss	ZERO
				goto	modb_r1					; ei, p�ris mitmes oli...
				movf	Char,W
;				movwf	mbCmnd					; seivib igax pettex
				sublw	modbus_cnf;modbus_wrmulti
				btfss	ZERO
				goto	modb_r00				; ei, �kki wr_multi ?
				movlw	.12;7
				movwf	countL
				goto	RREnd1					; j��b kuuldele...
modb_r00:		movf	Char,W
				sublw	modbus_wrmulti
				btfss	ZERO
				goto	modb_r1					; ei, siis ei n�pi
				movlw	.7
				movwf	countL
				goto	RREnd1					; j��b kuuldele...


modb_r0:;		movf	Char,W					; 1. bait on aadress. Kas 0x00 ehk broadcast ?
;;				movwf	mbAdr					; seivime
;				addlw	.0
;				btfsc	ZERO
;				goto	modb_r12				; on broadcast
;
;				movf	Char,W					; �kki k�sk 0x10 ?
;				sublw	modbus_wrmulti
;				btfss	ZERO
;				goto	modb_r1					; ei, siis ei n�pi - tavaline riid
;				movlw	.7
;				movwf	countL
;				goto	modb_r1

modb_r12:;		movlw	.12						; on broadcast, see peaks olema 12 -baidine
		;		movwf	countL
modb_r1:;		movf	bytecnt,W				; mitmes bait oli
		;		sublw	.3
		;		btfss	ZERO
		;		goto	modb_r14				; ei
		;		movf	Char,W
		;		movwf	work1;mbRAdrH					; seivi algusregistri aadress HIGH
modb_r14:;		movf	bytecnt,W				; mitmes bait oli
		;		sublw	.4
		;		btfss	ZERO
		;		goto	modb_r15				; ei
		;		movf	Char,W
		;		movwf	work2;mbRAdrL					; seivi algusregistri aadress LOW
modb_r15:;		movf	bytecnt,W				; mitmes bait oli
		;		sublw	.5
		;		btfss	ZERO
		;		goto	modb_r16				; ei
		;		movf	Char,W
;				movwf	mbnregH					; seivi regitrite arv HIGH (k�sud RD (3) ja write multi (10)
modb_r16:;		movf	bytecnt,W				; mitmes bait oli
		;		sublw	.6
		;		btfss	ZERO
		;		goto	modb_r17				; ei
		;		movf	Char,W
;				movwf	mbnregL					; seivi regitrite arv LOW (k�sud RD (3) ja write multi (10)
modb_r17:		movf	bytecnt,W
				subwf	countL,W				; saadetis k�es (countL-s oodatav baitide arv)?
				btfss	ZERO
				goto	RREnd1					; eip !
;				movf	mbCmnd,W				; kas oli k�sk 0x10 (kirjuta mitu reg. korraga)?
				movff	Puhver+.1,WREG
				sublw	modbus_wrmulti
				btfss	ZERO
				goto	modb_r2					; ei, pakett k�es, kontrolli summat
				btfsc	cmd10
				goto	modb_r2
				movff	Puhver+.6,WREG			; jah, loeme saadetavate baitide arvu ja ootame nende saabumist
				addlw	.2
				addwf	countL,F
				bsf		cmd10
				goto	RREnd1					; j��b kuuldele...

modb_r2:		movlw	0xFF					; pakett k�es, kontrollime
				movwf	_RS485chkH				; valmistume kontrollsumma arvutamiseks	
				movwf	_RS485chk
				decf	countL,F
				decf	countL,W
				movwf	bytecnt
				LFSR	.0,Puhver				; kontrollime summat

				bcf		Dir						; lisa Dir signaal nulli
				bcf		PIR3,CCP2IF
				bsf		PIE3,CCP2IE				; ja lubame j�lle sellise tekiamise

modb_r3:		movf	INDF0,W
				call	mb_crc16				; kontrollsummeeri
				incf	FSR0L,F
				decfsz	 bytecnt
				goto	modb_r3
				movf	_RS485chk,W				; kontroll
				subwf	INDF0,W
				btfss	ZERO
				goto	reset_ser2				; viga
				incf	FSR0L,F
				movf	_RS485chkH,W			; kontroll
				subwf	INDF0,W
				btfss	ZERO
				goto	reset_ser2				; viga, eksiteerib via reset_ser
;				call	reset_ser				; pakett ok, nulli ikkagi seriali side
				bsf		cmd_ok					; aga m�rgi �ra, et pakett oli ok
				bcf		SerialTimerOn			; taimer seisma
				banksel	PIE1
				bcf		PIE1,RC1IE				; enne uut k�sku vastuv�tu ei v�ta kuni senine t�idetud
				banksel	.0
				return
;===============================================
; ********* k�skude t�itmine *******************
;===============================================
command:		LFSR	.0,Puhver				; pakett OK, t�idame k�su !
				movff	Register274+.1,WREG		; 1. bait on slave aadress. Kas jutt mulle ?
				subwf	INDF0,W
				btfsc	ZERO
				goto	rcv_1					; jutt minule
				bcf		broadcast
				movf	INDF0,W					; kas broadcast (adr. 0x00) ?
				addlw	.0
				btfss	ZERO
				goto	reset_ser1				; viga
				bsf		broadcast				; edaspidi kuulame broadcast-k�ske ka kuid neile ei vasta !
;			btfsc	cmd10
;			goto	rcv_1					; broadcastile vastab vaid siis kui oli k�sk konfiks
			call	reload_side				; oli midagi muud, teeme ikkagi sidetaimeri reload aga ei vasta
			goto	reset_ser1				; 
;----------- CHG side reload iga baidi kuulmisel ------------------
rcv_1:			call	reload_side				; sidetaimeri reload
;----------- CHG side reload iga baidi kuulmisel ------------------
				incf	FSR0L,F
				movf	INDF0,W					; kas oli k�sk RD holding register (0x03) ?
				sublw	modbus_rd
				btfsc	ZERO
				goto	modb_read				; jah
				movf	INDF0,W					; kas oli k�sk WR holding register (0x06) ?
				sublw	modbus_wr
				btfsc	ZERO
				goto	modb_write				; jah
				movf	INDF0,W					; kas oli k�sk WR conf register (0x45) ?
				sublw	modbus_cnf
				btfsc	ZERO
				goto	modb_conf				; jah
				movf	INDF0,W					; kas oli k�sk WR multiple registers (0x10) ?
				sublw	modbus_wrmulti
				btfsc	ZERO
				goto	modb_multikulti			; jah
				goto	valekask				; teavita et oli vale k�sk
;===============================================
; sidetaimeri reload
;===============================================
reload_side:	bsf		sidetmron
				movlw	sidetime
				movwf	sidetaimer
				movff	Register277,WREG		; taasta pulsi kestus
				movwf	reset1pulsetmr		
				movff	Register277+.1,WREG		; taasta side kadumise viiteaeg reseti 1 generaatorile
				movwf	reset1dlytmr
;				bsf		reset1					; reseti 1 pinn maha (igaks juhux)
			bsf		n_reset1				; inversioon
				bcf		reset1pulseon			; reseti 1 pulsi generaator OHV (igaks juhux)
rls1:;			bsf		pank1
				movff	Register279+.0,WREG		; taasta pulsi kestus
				movwf	reset2pulsetmr			
				movff	Register279+.1,WREG		; side kadumise viiteaeg reseti 2 generaatorile
				movwf	reset2dlytmr
				bcf		PWRSW					; reseti 2 pinn maha (igaks juhux)
				bcf		reset2pulseon			; reseti 2 pulsi generaator OHV (igaks juhux)
rls2:			return
;===============================================
; loe N registrit
;===============================================
modb_read:		bcf		write
				call	validate				; kas aadress ja loetav pikkus piirides ?
				btfss	CARRY
				goto	mbr0
				bcf		clrsticky	
				goto	valedata				; ei ole, nii �tlegi

mbr0:			call	ch_algadr				; kas loetava algusaadress on piirides ?
				btfsc	CARRY
				goto	valedata				; ei ole, nii �tlegi
				btfsc	broadcast				; braodcast'i puhul ei vasta !
				goto	reset_ser				; muide, broadcast'iga lugeda olex eriti totter aga olgu piirang ikkagi - lolle leidub...

				bsf		clrsticky				; oletame, et lubataxe sticky't maha v�tta
				btfss	reg0					; kas vaid 1 register & reg0 ? siis clrsticky =0 !!!
				goto	mbr00					; ei
				movff	n_regL,WREG				; loetavate registrite arv (LOW)
				sublw	.1
				btfss	ZERO
				goto	mbr00
				bcf		clrsticky				; jah, siis oli vaid reg. 0 lugemine => ei luba sticky't maha v�tta

mbr00:			btfss	clrsticky				; lugesime reg.1 ? (suvalises kombinatsioonis teistega)
				goto	mbr1
				movlw	0xFF					
				movf	dinpress
				clrf	dinsticky				; jah, siis nullime vastava sticky-baidi sest master sai teavitet
				clrf	ainsticky				; jah, siis nullime vastava sticky-baidi sest master sai teavitet
				bcf		clrsticky				
mbr1:			bcf		writeit					; oli lugemine, ei ole vaja kirjutada
				btfsc	pwm						; loeti pwmi registreid ?
				goto	mbr2					; jah, FSR0 on juba laetud
				LFSR	.0,Register0
				movff	m_radrL,serpartmp
				rlcf	serpartmp,W				; liidame puhvri alguse (aadress *2)
				addwf	FSR0L,F
				btfsc	CARRY	
				incf	FSR0H,F					; loetava registri aadress olemas !
mbr2:			movff	n_regH,WREG				; loetavate registrite arv (HIGH)
				movwf	countH
				movff	n_regL,serpartmp
				rlcf	serpartmp,W				; loetavate registrite arv (LOW)
				movwf	countL
; Vastus: ADR, CMND, NoB, DataH,DataL,..., CRCH, CRCL				
				call	paketialgus				; kirjuta paketi algus (ADR, CMND) puhvrisse	
				movf	countL,W
				movwf	POSTINC1				; NOB
				call	mb_crc16
				incf	bytecnt,F				; loendame baite
				btfss	dallas					; kas dallase asjad v�i loendid ?
				goto	modb_read_loop
				movlw	.164;5
				addwf	FSR0L,F
				btfsc	CARRY
				incf	FSR0H,F
modb_read_loop:	movf	POSTINC0,W
				movwf	POSTINC1
				call	mb_crc16
				incf	bytecnt,F				; loendame baite
				decfsz	countL
				goto	modb_read_loop
				btfss	reg10					; kui loeti registrit 10, v�ib n��d wiegandi bitiloendid nullida
				goto	modb_read_end
				btfss	wiegandAto				; wiegand A tulemus k�es ?
				goto	modb_r_1				; eip
				clrf	WAbitcount				; jah - nulli A-wiegandi bitiloendid
				bcf		wiegandAto				; n��d lubame uuesti A-wiegandi lugemise
				clrf	Register10+.0
				clrf	Register11+.0
				clrf	Register11+.1
				clrf	Register12+.0
				clrf	Register12+.1
				clrf	Register13+.0
				clrf	Register13+.1
				clrf	Register14+.0
				clrf	Register14+.1
modb_r_1:		btfss	wiegandBto				; wiegand B tulemus k�es ?
				goto	modb_read_end			; eip
				clrf	WBbitcount				; jah - nulli B-wiegandi bitiloendid
				bcf		wiegandBto				; n��d lubame uuesti B-wiegandi lugemise
				clrf	Register10+.1
				clrf	Register15+.0
				clrf	Register15+.1
				clrf	Register16+.0
				clrf	Register16+.1
				clrf	Register17+.0
				clrf	Register17+.1
				clrf	Register18+.0
				clrf	Register18+.1
modb_read_end:	goto	paketilopp				; kirjuta CRC puhvrisse, loenda baite ja saada vastus teele
;===============================================
; kirjuta �hte registrit
;===============================================
modb_write:		bsf		write
				call	validate				; kas aadress ja loetav pikkus piirides ?
				btfsc	CARRY
				goto	valedata				; ei ole, nii �tlegi
				call	ch_algadr				; kas kirjutatava algusaadress on piirides ja kas lubatud kirjutada ?
				btfsc	CARRY
				goto	valedata				; ei ole, nii �tlegi
				btfsc	readonly				; kirjutamine lubatud ?
				goto	valedata				; ei ole, nii �tlegi
				btfsc	pwm						; loeti pwmi registreid ?
				goto	modb_write0				; jah, FSR0 on juba laetud
				LFSR	.0,Register0
				movff	m_radrL,serpartmp
				rlcf	serpartmp,W				; liidame puhvri alguse (aadress *2)
				addwf	FSR0L,F
				btfsc	CARRY	
				incf	FSR0H,F					; loetava registri aadress olemas !
				btfss	dallas					; kas loendid ?
				goto	modb_write0				; eip !
				movlw	.165					; jah, need nati kaugemal
				addwf	FSR0L,F
				btfsc	CARRY
				incf	FSR0H,F

modb_write0:	movff	n_regH,WREG				; kirjutab data (HIGH)
				movwf	POSTINC0
				btfsc	sync					; PWMi kirjutame ka t��registrisse
				movwf	POSTINC1
				movff	n_regL,WREG				; kirjutab data (LOW)
				movwf	INDF0
				btfsc	sync
				movwf	POSTINC1
				btfss	reg0					; oli register 0 ?
				goto	modb_write1				; eip !
				movff	Register0,WREG			; jah, dubleerime kohe porti ka
				movwf	DoutPort				; digiv�ljundid ja PU-d
				movff	Register0+.1,Registertemp7	
;----
				andlw	0x0F
				movwf	PORTA					; ja bitikaupa miskip�rast ei l�he !?
				btfsc	Registertemp7,.4		; kombineerime baidid ANA-pordi juhtimisex	
				bsf		PORTA,.5
;				addlw	0x20					; analoog v�i sisendi puhul kirjutamine nagunii ei m�ju
				bcf		CARRY
				rrcf	Registertemp7,F
				swapf	Registertemp7,W
				andlw	0x07
				movwf	PORTE

modb_write1:	bcf		reg0				
				btfss	res_pwm					; kirjutati reg.149 v�i 150-sse ?
				goto	modb_write2
				call	reset_pwm				; jep! reseti PWM !
				call	reset_per				; ja tee periood nullix ja v�ljundid kah !
modb_write2:	btfsc	broadcast				; braodcast'i puhul ei vasta !
				goto	reset_ser
; Vastus: ADR, CMND, ADRH,ADRL,DATAH,DATAL, CRCH,CRCL
				call	paketialgus				; kirjuta paketi algus (ADR, CMND) puhvrisse	
xyz:			movff	Puhver+RSBufLen-.2,WREG;Puhver+.2,W
				movwf	POSTINC1
				call	mb_crc16
				incf	bytecnt,F				; loendame baite
				movff	Puhver+RSBufLen-.1,WREG;Puhver+.3,W
				movwf	POSTINC1
				call	mb_crc16
				incf	bytecnt,F				; loendame baite
				movff	n_regH,WREG				; data HIGH
				movwf	POSTINC1
				call	mb_crc16
				incf	bytecnt,F				; loendame baite
				movff	n_regL,WREG				; data LOW
				movwf	POSTINC1
				call	mb_crc16
				incf	bytecnt,F				; loendame baite
				goto	paketilopp				; kirjuta CRC puhvrisse, loenda baite ja saada vastus teele
;===============================================
; kirjuta mitu registrit
;===============================================
modb_multikulti:movff	m_radrL,WREG			; multi-kultit lubame vaid PWMi regitritele			
				sublw	.100-.1					; kas <100 ?
				btfsc	CARRY
				goto	valedata				; oli liiga madal aadress
				movff	m_radrL,WREG				
				sublw	.115					; kas >115 ?
				btfss	CARRY
				goto	valedata				; oli liiga k�rge aadress
				movff	m_radrL,serpartmp		; oli PWMi register
				movlw	.100
				subwf	serpartmp,W
				movff	WREG,serpartmp
				LFSR	.0,pwm0set				; kirjutame nii set kui ka t��registrisse !
				LFSR	.1,pwm0work
				bcf		CARRY
				rlcf	serpartmp,W				; liidame puhvri alguse (aadress *2)
				addwf	FSR0L,F
				btfsc	CARRY	
				incf	FSR0H,F					; loetava registri aadress olemas !
				bcf		CARRY
				rlcf	serpartmp,W				; liidame puhvri alguse (aadress *2)
				addwf	FSR1L,F
				btfsc	CARRY	
				incf	FSR1H,F					; t��registri aadress samuti olemas !
; kontrollime kirjutatava pikkust
				movff	FSR0H,mb_temp1
				movff	FSR0L,mb_temp2
				movff	n_regH,WREG				; loetavate registrite arv (HIGH)
				addlw	.0
				btfss	ZERO
				goto	valedata				; liiga pikk kirjutamise soov -> per...
				movff	n_regL,WREG				; loetavate registrite arv (LOW)
				bcf		CARRY
				addwf	mb_temp2,F
				btfsc	CARRY
				incf	mb_temp1,F
				movlw	HIGH(Register150)
				cpfsgt	mb_temp1
				goto	multik1
				goto	valedata
multik1:		cpfseq	mb_temp1
				goto	multik2					; on OK !
				movlw	LOW(Register150)
				cpfsgt	mb_temp2
				goto	multik2					; on OK !
				goto	valedata
multik2:		movf	n_regH,W				; kirjutavate registrite arv (HIGH)
				movwf	countH
				movff	n_regL,countL
				rlcf	countL,F				; kirjutavate registrite arv (LOW)
				movf	countH,W				; kui kirjutatava pikkus =0, siis per...
				addlw	.0
				sublw	.0
				btfss	ZERO
				goto	modb_mk
				movf	countL,W				
				addlw	.0
				sublw	.0
				btfsc	ZERO
				goto	valedata
modb_mk:		LFSR	.2,Puhver+.7			; daata tuleb siit 
modb_mkloop:	movff	INDF2,POSTINC1
				movff	POSTINC2,POSTINC0
				decfsz	countL
				goto	modb_mkloop
				btfsc	broadcast				; broadcast'i puhul ei vasta !
				goto	reset_ser				
; Vastus: ADR, CMND, ST_REGH, ST_REGL, No_of_regH,No_of_reg_L, CRCH, CRCL
				call	paketialgus				; kirjuta paketi algus (ADR, CMND) puhvrisse	
				movff	m_radrH,WREG
				movwf	POSTINC1
				call	mb_crc16
				incf	bytecnt,F				; loendame baite
				movff	m_radrL,WREG
				movwf	POSTINC1
				call	mb_crc16
				incf	bytecnt,F				; loendame baite
				movff	n_regH,WREG				; data HIGH
				movwf	POSTINC1
				call	mb_crc16
				incf	bytecnt,F				; loendame baite
				movff	n_regL,WREG				; data LOW
				movwf	POSTINC1
				call	mb_crc16
				incf	bytecnt,F				; loendame baite
				goto	paketilopp				; kirjuta CRC puhvrisse, loenda baite ja saada vastus teele
;===============================================
; kirjuta uus konf
;===============================================
;mb(1)=0   ' Broadcast
;mb(2)=69  ' Set config
;mb(3)=eser1(mba)   'Serial number, part 1
;mb(4)=eser2(mba)   'Serial number, part 2
;mb(5)=eser1(mba)   'Serial number, part 1 again
;mb(6)=eser2(mba)   'Serial number, part 2 again
;mb(7)=imask(mba) ' reg 273 = ADDR 272, pullup.
;mb(8)=0 '  reg274=ADDR 273, 0=19k2e1, 128=19k8n1 - bit 7 parity,
;     ' mb(8)=24 'sticky bit 8 + debounce off 16! loendite jaoks.
;mb(9)=mba ' REG 275, ADDR 274, Modbus address
;mb(10)=omask(mba) ' reg 276=ADDR 275, io suund, omask=valjundid.
;    'mis siin maskis puuduvad, on sisendid! kas ai voi di, pole sel
;juhul oluline, kui nad ka paralleelselt loetavad oma registritest
modb_conf:		bcf		reg0
				bsf		write				
				incf	FSR0L,F					; viita seerianumbrile (HIGH)
				movff	Register258+.1,WREG		; �ige ?
				subwf	INDF0,W
				btfss	ZERO
				goto	reset_ser				; jama !
				incf	FSR0L,F					; viita seerianumbrile (LOW)
				movff	Register259+.1,WREG		; �ige ?
				subwf	INDF0,W
				btfss	ZERO
				goto	reset_ser				; jama !
				bsf		writeit					; paneme peale vastamist parameetrid kirja ka !
				incf	FSR0L,F					; �le ser. nr. korduse
				incf	FSR0L,F					; �le ser. nr. korduse
				incf	FSR0L,F					; �le ser. nr. korduse
				movf	POSTINC0,W				; pull-up mask
				movff	WREG,Register272+.1
				movlw	0x00
				movff	WREG,Register272
				movf	POSTINC0,W				; seriali parameetrid
				movff	WREG,Register273+.1		; kohe RAM'i kirja, save_setup kirjutab EEPROMi
				movlw	0x00
				movff	WREG,Register273
				movff	Register274+.1,serpartmp; ajutine seiv
				movf	POSTINC0,W				; modbussi aadress
;				movff	WREG,Register274+.1
				movlw	0x00
				movff	WREG,Register274
				movf	INDF0,W					; IO suund
				movff	WREG,Register275+.1
				movlw	0x00
				movff	WREG,Register275
				btfsc	broadcast				; braodcast'i puhul ei vasta !
				goto	reset_ser
; Vastus: ADR, CMD, ID1, ID2, ID1, ID2, CRCH,CRCL
				call	paketialgus				; kirjuta paketi algus (ADR, CMND) puhvrisse	
				movff	serpartmp,Register274+.1; j�ustame uue aadressi
				movff	Register258+.1,WREG		; serial nr.  HIGH
				movwf	POSTINC1
				call	mb_crc16
				incf	bytecnt,F				; loendame baite
				movff	Register259+.1,WREG		; serial nr.  LOW
				movwf	POSTINC1
				call	mb_crc16
				incf	bytecnt,F				; loendame baite
				movff	Register258+.1,WREG		; serial nr.  HIGH
				movwf	POSTINC1
				call	mb_crc16
				incf	bytecnt,F				; loendame baite
				movff	Register259+.1,WREG		; serial nr.  LOW
				movwf	POSTINC1
				call	mb_crc16
				incf	bytecnt,F				; loendame baite
;===============================================================================
; Abifunktsioonid
;===============================================================================
paketilopp:		movf	_RS485chk,W				; ja summa
				movwf	POSTINC1
				incf	bytecnt,F				; loendame baite
				movf	_RS485chkH,W	
				movwf	POSTINC1
				incf	bytecnt,F				; loendame baite
				call	dly						; 3,98 ms viidet
				LFSR	.1,Puhver				; hakkab saatma
				bsf		Dir
send1:			movf	POSTINC1,W
				call	SendCHAR
				decfsz	bytecnt
				goto	send1
				bcf		Dir
;				btfsc	res_pwm					; kirjutati reg.149 v�i 150-sse ?
;				call	reset_pwm				; jep! reseti PWM !
;				btfsc	res_pwm					; kirjutati reg.150-sse ?
;				call	reset_per				; jep! tee periood nullix ja v�ljundid kah !
				btfss	writeit					; oli konfidaata ja see tuleks EEPROMi kirjutada?
				goto	reset_ser
				bcf		INTCON,GIE
			bcf		INTCON,PEIE
				call	Save_Setup				; jah
				bsf		INTCON,GIE
			bsf		INTCON,PEIE

send2:			bcf		writeit					; kirjutatud !
				goto	reset_ser;1				; l�petame jama ��...
;===============================================================================
paketialgus:	LFSR	.1,Puhver				; formeerime vastuse saatepuhvrisse
				movlw	0xFF					
				movwf	_RS485chkH				; valmistume kontrollsumma arvutamiseks	
				movwf	_RS485chk
				clrf	bytecnt
				movff	Register274+.1,WREG		; oma aadress
				movwf	POSTINC1
				call	mb_crc16
				incf	bytecnt,F				; loendame baite
				movf	POSTINC1,W				; k�sk juba kirjas, arvutame ta CRC sisse ja loendame baite kah
				call	mb_crc16
				incf	bytecnt,F				; loendame baite
				return
;===============================================================================
dly:			movlw	T3resoL					; lae taimer 3 (katkestus iga 5,5 mS tagant)
				movwf	TMR3L
				movlw	T3resoH
				movwf	TMR3H
				bcf		PIR2,TMR3IF
				bcf		PIE2,TMR3IE
				movlw	0x31					; 1:8 prescaler, GO !
				movwf	T3CON					
dly1:			btfss	PIR2,TMR3IF					; 5,5 mS viidet
				goto	dly1
				movlw	0x00
				movwf	T3CON					
				bcf		PIR2,TMR3IF
				bcf		PIE2,TMR3IE
				return
;;===============================================================================
;dly:			movlw	0x00
;				movwf	TMR4
;				bsf		T4CON,TMR4ON
;				bcf		PIR4,TMR4IF
;dly1:			btfss	PIR4,TMR4IF					; 5,5 mS viidet
;				goto	dly1
;				bcf		T4CON,TMR4ON
;				bcf		PIR4,TMR4IF
;				bcf		PIE4,TMR4IE
;				return
;;===============================================================================
;===============================================================================
;if( X <= K ) (signed)
;  movlw Khi
;  movwf Yhi
;  movlw Klo
;  movwf Ylo
;  call compare_signed_16
;  skpc
;  goto endif
; then:
;...
;endif: 

;    * X = XH:XL the Hi and Lo bytes of the RAM variable X,
;    * Y = YH:YL the Hi and Lo bytes of the RAM variable Y, and
;    * K = KH:KL the Hi and Lo bytes of some fixed constant value K. 
compare16_16: ; 7
				movf XH,W
				subwf YH,W 						; subtract Y-X
Are_they_equal:	btfss	ZERO					; Are they equal ?
	    		goto results16
				movf XL,W						; yes, they are equal -- compare lo				
				subwf YL,W						; subtract Y-X
results16:
				; if X=Y then now Z=1.
				; if Y<X then now C=0.
				; if X<=Y then now C=1.
				return
;===============================================================================
validate:		movff	m_radrH,XH				; adr >18 ?
				movff	m_radrL,XL
				movlw	HIGH(.18)
				movwf	YH
				movlw	LOW(.18)
				movwf	YL
				call	compare16_16
				btfss	CARRY			
				goto	validate1
				movlw	HIGH(.20)				; adr <=18 -> max=20
				movwf	XH
				movlw	LOW(.20)
				movwf	XL
				goto	validate_end
validate1:		movff	m_radrH,XH				; adr >115 ?
				movff	m_radrL,XL
				movlw	HIGH(.115)
				movwf	YH
				movlw	LOW(.115)
				movwf	YL
				call	compare16_16
				btfss	CARRY			
				goto	validate2
				movlw	HIGH(.99)				; adr <100 ?
				movwf	YH
				movlw	LOW(.99)
				movwf	YL
				call	compare16_16
				btfsc	CARRY
				goto	validate_bad			; siis error !			
				movlw	HIGH(.117)				; 100 >= adr <=115 -> max=117 -> I/O registrid
				movwf	XH
				movlw	LOW(.117)
				movwf	XL
				goto	validate_end
validate2:		movff	m_radrH,XH				; adr >151 ?
				movff	m_radrL,XL
				movlw	HIGH(.151)
				movwf	YH
				movlw	LOW(.151)
				movwf	YL
				call	compare16_16
				btfss	CARRY			
				goto	validate3
				movlw	HIGH(.148)				; adr <149 ?
				movwf	YH
				movlw	LOW(.148)
				movwf	YL
				call	compare16_16
				btfsc	CARRY
				goto	validate_bad			; siis error !			
				movlw	HIGH(.153)				; max=153 -> PWMi konfiregistrid
				movwf	XH
				movlw	LOW(.153)
				movwf	XL
				goto	validate_end
validate3:		movff	m_radrH,XH				; adr >259 ?
				movff	m_radrL,XL
				movlw	HIGH(.259)
				movwf	YH
				movlw	LOW(.259)
				movwf	YL
				call	compare16_16
				btfss	CARRY			
				goto	validate4
				movlw	HIGH(.255)				; adr <256 ?
				movwf	YH
				movlw	LOW(.255)
				movwf	YL
				call	compare16_16
				btfsc	CARRY
				goto	validate_bad			; siis error !			
				movlw	HIGH(.261)				; max=261 -> konfiregistrid
				movwf	XH
				movlw	LOW(.261)
				movwf	XL
				goto	validate_end
validate4:		movff	m_radrH,XH				; adr >279 ?
				movff	m_radrL,XL
				movlw	HIGH(.279)
				movwf	YH
				movlw	LOW(.279)
				movwf	YL
				call	compare16_16
				btfss	CARRY			
				goto	validate5
				movlw	HIGH(.269)				; adr <270 ?
				movwf	YH
				movlw	LOW(.269)
				movwf	YL
				call	compare16_16
				btfsc	CARRY
				goto	validate_bad			; siis error !			
				movlw	HIGH(.281)				; max=281 -> konfiregistrid
				movwf	XH
				movlw	LOW(.281)
				movwf	XL
				goto	validate_end
validate5:		movff	m_radrH,XH				; adr >415 ?
				movff	m_radrL,XL
				movlw	HIGH(.415)
				movwf	YH
				movlw	LOW(.415)
				movwf	YL
				call	compare16_16
				btfss	CARRY			
				goto	validate6
				movlw	HIGH(.399)				; adr <400 ?
				movwf	YH
				movlw	LOW(.399)
				movwf	YL
				call	compare16_16
				btfsc	CARRY
				goto	validate_bad			; siis error !			
				movlw	HIGH(.417)				; max=417 -> loendid
				movwf	XH
				movlw	LOW(.417)
				movwf	XL
				goto	validate_end
validate6:		movff	m_radrH,XH				; adr >499 ?
				movff	m_radrL,XL
				movlw	HIGH(.499)
				movwf	YH
				movlw	LOW(.499)
				movwf	YL
				call	compare16_16
				btfss	CARRY			
				goto	validate7
				movlw	HIGH(.497)				; adr <498 ?
				movwf	YH
				movlw	LOW(.497)
				movwf	YL
				call	compare16_16
				btfsc	CARRY
				goto	validate_bad			; siis error !			
				movlw	HIGH(.501)				; max=501 -> PWMi konfiregistrid
				movwf	XH
				movlw	LOW(.501)
				movwf	XL
				goto	validate_end
validate7:		movff	m_radrH,XH				; adr >608 ?
				movff	m_radrL,XL
				movlw	HIGH(.608)
				movwf	YH
				movlw	LOW(.608)
				movwf	YL
				call	compare16_16
				btfss	CARRY			
				goto	validate8
				movlw	HIGH(.599)				; adr <600 ?
				movwf	YH
				movlw	LOW(.599)
				movwf	YL
				call	compare16_16
				btfsc	CARRY
				goto	validate_bad			; siis error !			
				movlw	HIGH(.610)				; max=610 -> DS18B20 n�idud
				movwf	XH
				movlw	LOW(.610)
				movwf	XL
				goto	validate_end
validate8:		movff	m_radrH,XH				; adr >685 ?
				movff	m_radrL,XL
				movlw	HIGH(.685)
				movwf	YH
				movlw	LOW(.685)
				movwf	YL
				call	compare16_16
				btfss	CARRY			
				goto	validate9
				movlw	HIGH(.649)				; adr <650 ?
				movwf	YH
				movlw	LOW(.649)
				movwf	YL
				call	compare16_16
				btfsc	CARRY
				goto	validate_bad			; siis error !			
				movlw	HIGH(.687)				; max=687 -> DS18B20 ID-d
				movwf	XH
				movlw	LOW(.687)
				movwf	XL
				goto	validate_end
validate9:		movff	m_radrH,XH				; adr >731 ?
				movff	m_radrL,XL
				movlw	HIGH(.731)
				movwf	YH
				movlw	LOW(.731)
				movwf	YL
				call	compare16_16
				btfss	CARRY			
				goto	validate10
				movlw	HIGH(.699)				; adr <700 ?
				movwf	YH
				movlw	LOW(.699)
				movwf	YL
				call	compare16_16
				btfsc	CARRY
				goto	validate_bad			; siis error !			
				movlw	HIGH(.733)				; max=733 -> DS2438 n�idud
				movwf	XH
				movlw	LOW(.733)
				movwf	XL
				goto	validate_end
validate10:		movff	m_radrH,XH				; adr >785 ?
				movff	m_radrL,XL
				movlw	HIGH(.785)
				movwf	YH
				movlw	LOW(.785)
				movwf	YL
				call	compare16_16
				btfss	CARRY			
				goto	validate11
				movlw	HIGH(.749)				; adr <750 ?
				movwf	YH
				movlw	LOW(.749)
				movwf	YL
				call	compare16_16
				btfsc	CARRY
				goto	validate_bad			; siis error !			
				movlw	HIGH(.787)				; max=787 -> DS2438 ID'd
				movwf	XH
				movlw	LOW(.787)
				movwf	XL
				goto	validate_end
validate11:		movff	m_radrH,WREG			; adr =999 ?
				sublw	HIGH(.999)
				btfss	ZERO
				goto	validate_bad			; siis error !			
				movff	m_radrL,WREG
				sublw	LOW(.999)
				btfss	ZERO
				goto	validate_bad			; siis error !			
				goto	validate_ok				; korras !
validate_end:	movff	Puhver+.1,WREG			; registrite arv anti vaid k�skude read (0x03) ja wr_multi (0x10) puhul
				sublw	modbus_wr
				btfsc	ZERO
				goto	validate_ok				; k�sk WRITE, siis reg. arvu ei anta, on OK
				movff	Puhver+.1,WREG			
				sublw	modbus_cnf
				btfsc	ZERO
				goto	validate_ok				; k�sk CONF, siis reg. arvu ei anta, on OK
				movff	m_radrL,YL				; ADR+N_REG -> Y
				movff	m_radrH,YH
				movff	n_regL,WREG
				addwf	YL,F
				btfsc	CARRY
				incf	YH,F
				movff	n_regH,WREG
				addwf	YH,F
				call	compare16_16			; kui Y>=X, siis error !
				btfsc	CARRY
				goto	validate_bad
validate_ok:	bcf		CARRY
				return
validate_bad:	bsf		CARRY
				return		
;===============================================================================

ch_algadr:		bcf		clrsticky				; oletame, et ei loetud reg.1-e.
				bcf		reg0
				bcf		reg10					; ja et ei loetud ka reg. 10-t					
				bcf		dallas
				bcf		pwm
				bcf		sync
				bcf		res_pwm
				bcf		readonly
			movff	m_radrL,WREG
			movff	WREG,Puhver+RSBufLen-.1
			movff	m_radrH,WREG				; kas register 0 ?
			movff	WREG,Puhver+RSBufLen-.2
				addlw	.0
				btfss	ZERO
				goto	ch_alg0;1
				movff	m_radrL,WREG
				addlw	.0
				btfss	ZERO
				goto	ch_alg0;1
				bsf		reg0					; et alustati registrist 0
;				bcf		readonly
				goto	chk_algok				; on r/w register 0, k�ik OK
ch_alg0:		movff	m_radrL,WREG			; aadressi kontroll j�tkub
				sublw	0xFF					; kas device type register ?
				btfsc	ZERO
				goto	chk_alg3				; jah !
				movff	m_radrH,WREG
				addlw	.0
				btfss	ZERO
				goto	chk_alg3				; aadress > 256, vaata kas konfiregister	
				movff	m_radrL,WREG			; kas tavaregistrid ehk adr. <18 ?
				sublw	workregisters-.1
				btfsc	CARRY
				goto	chk_alg2				; jep !

				movff	m_radrL,WREG			; kas register 149 ?
				sublw	0x95
				btfsc	ZERO
				goto	chk_alg12b
				movff	m_radrL,WREG			; aadressi kontroll j�tkub
				sublw	0x96					; kas pwmi registrid 100...115,150 ?
				btfsc	ZERO
				goto	chk_alg12				; oli register 150
				movff	m_radrL,WREG
				sublw	0x97					; 
				btfsc	ZERO
				goto	chk_alg12a				; oli register 151
				btfss	CARRY
				goto	chk_algbad				; oli liiga k�rge aadress
				movff	m_radrL,WREG				
				sublw	.100-.1					; kas <100 ?
				btfsc	CARRY
				goto	chk_algbad				; oli liiga madal aadress
				movff	m_radrL,WREG				
				sublw	.115					; kas >115 ?
				btfss	CARRY
				goto	chk_algbad				; oli liiga k�rge aadress
				goto	chk_alg11				; pwmi register oli

;				goto	chk_alg2				; t��registrid, kontrolli et adr <= 18	
chk_alg1:		bsf		readonly				; on vaid loetav register
				movlw	.0
				movff	WREG,m_radrH
;			clrf	m_radrH					; k�rgemate adr. puhul vajalik
				goto	chk_algok				; OK
;chk_alg3:		movlw	.2
;				subwf	m_radrH,W
;				btfsc	ZERO
;				goto	chk_alg8				; tegu Dallase juraga
;				movlw	.3
;				subwf	m_radrH,W
;				btfsc	ZERO
;				goto	chk_alg8				; tegu Dallase juraga aga ilmselt DS2438
chk_alg3:		movff	m_radrH,WREG
				sublw	.2
				btfsc	ZERO
				goto	chk_alg8				; tegu Dallase juraga
				movff	m_radrH,WREG
				sublw	.3
				btfsc	ZERO
				goto	chk_alg8				; tegu Dallase juraga aga ilmselt DS2438
;**** uptime loendamine ****
;chk_alg3c:		movlw	.1						; kas uptime loendi (1F2,3,4,5)
;				subwf	m_radrH,W
;				btfss	ZERO
;				goto	chk_alg3e				; eip
;				movlw	0xF2
;				subwf	m_radrL,W
;				btfss	CARRY
;				goto	chk_alg3e				; eip
;				movlw	0x5D
;				movwf	m_radrL
;				clrf	m_radrH					
;				bsf		dallas
;				goto	chk_alg1				; jah, see on r/0 register !
chk_alg3c:		movff	m_radrH,WREG			; kas uptime loendi (1F2,3,4,5)
				sublw	.1
				btfss	ZERO
				goto	chk_alg3e				; eip
				movff	m_radrL,serpartmp
				movlw	0xF2
				subwf	serpartmp,W
				btfss	CARRY
				goto	chk_alg3e				; eip
				movlw	0x5D
				movff	WREG,m_radrL
				movlw	.0
				movff	WREG,m_radrH					
				bsf		dallas
				goto	chk_alg1				; jah, see on r/0 register !
;**** uptime loendamine ****
chk_alg3e:		movff	m_radrH,serpartmp
				movlw	.1
				subwf	serpartmp,W
				btfss	CARRY
				goto	chk_alg3b				; >100h , <200h
				bsf		dallas					; ei ole dallase asi aga on loendi mis asub samuti taamal...
				movff	m_radrL,WREG			; on loendid, algavad 0x190-st
				sublw	0x90-.1						
				btfss	CARRY
				goto	chk_alg7;6				; Loendid on r/w registrid

chk_alg3b:		bcf		dallas					; ei ole dallase asi ei ole loendi kah, parandame
				movff	m_radrL,WREG			; reg. >= 256, kas 0x0100, 0x0102 v�i 0x0103 ?
				sublw	0x00
				btfss	ZERO
				goto	chk_alg3a
				movlw	.19						; 256dec -> jrk.nr. 19dec , r-only, dev. type
				movff	WREG,m_radrL
				goto	chk_alg1
;-
chk_alg3a:		movff	m_radrL,WREG	
				sublw	0x01
				btfss	ZERO
				goto	chk_alg3d
				movlw	.20						; 257 -> 20, r-only, firmware nr.
				movff	WREG,m_radrL
				goto	chk_alg1

chk_alg3d:		movff	m_radrL,WREG				
				sublw	0x02
				btfss	ZERO
				goto	chk_alg4
				movlw	.21						; 258 -> 21, r-only, serial High
				movff	WREG,m_radrL
				goto	chk_alg1
chk_alg4:		movff	m_radrL,WREG				
				sublw	0x03
				btfss	ZERO
				goto	chk_alg5
				movlw	.22						; 259 -> 22, r-only, serial Low
				movff	WREG,m_radrL
				goto	chk_alg1
chk_alg5:		movff	m_radrL,WREG			; adr 261...270 v�lja !
				sublw	LOW(.270-.1)			; kas < 270 ?
				btfsc	CARRY
				goto	chk_algbad				; siis per...
				movff	m_radrL,WREG			; kas > 279;7 ?
				sublw	LOW(.280-.1)
				btfss	CARRY
				goto	chk_algbad				; siis per...
				bsf		writeit					; oli konfidaata.kirjutada EEPROMi !
; 
				movff	m_radrL,WREG			; kas = 273 ?
				sublw	LOW(.273)
				btfss	ZERO
				goto	chk_alg5a				; eip !
				bsf		reset1ena				; jah, siis teeme kohe ka reset 1-e
				movlw	.0
				movwf	reset1dlytmr
;
chk_alg5a:		movff	m_radrL,WREG
				addlw	.9;8
				movff	WREG,m_radrL
;				addwf	m_radrL,F				; aadress +8, high = 0
				movlw	.0
				movff	WREG,m_radrH
;				clrf	m_radrH					
chk_alg6:		bcf		readonly				; on r/w register
				goto	chk_algok

chk_alg2:;		movf	m_radrL,W				; tavaregistrid, kas adr <18 ?
		;		sublw	workregisters-.1
		;		btfss	CARRY
		;		goto	chk_algbad				; eip, per...
				bsf		clrsticky				; ja j�relikult v�ib sticky biti ka maha v�tta peale lugemist
				movff	m_radrL,WREG
				sublw	.10						; kas oli reg. 10 lugemine ?
				btfsc	ZERO
				bsf		reg10
				goto	chk_alg1				; jah, need k�ik read-only !
chk_alg7:		movff	m_radrL,serpartmp
				movlw	.112					; loendid
				subwf	serpartmp,W
				movff	WREG,m_radrL
				movlw	.0
				movff	WREG,m_radrH
				goto	chk_alg6				; need on k�ik r/w registrid

chk_alg8:		movff	m_radrH,WREG			; teha reset (�ksessiti reg. 999) ?
				sublw 	HIGH(.999)				
				btfss	ZERO
				goto	chk_alg8a				; ei
				movff	m_radrL,WREG
				sublw 	LOW(.999)				; teha reset (�ksessiti reg. 999) ?
				btfss	ZERO
				goto	chk_alg8a				; ei
; lubame reseti vaid wr k�su puhul kui kirjutati data 0xDEAD
				movff	Puhver+.1,WREG
				sublw	0x06
				btfss	ZERO
				goto	chk_algbad
				movff	n_regH,WREG
				sublw	0xDE
				btfss	ZERO
				goto	chk_algbad
				movff	n_regL,WREG
				sublw	0xAD
				btfss	ZERO
				goto	chk_algbad
				bcf		INTCON,GIE
;rese:			goto	rese					; jah, laseme vahikoeral reseti teha
				reset							; jah

chk_alg8a:		movff	m_radrH,WREG			; > 700 ? Siis DS2438 kivi
				sublw	.3
				btfsc	ZERO
				goto	chk_alg13				; ja kindlasti
				movff	m_radrH,WREG				
				sublw	.2
				btfss	ZERO
				goto	chk_alg8b				; ei peaks juhtuma...
				movff	m_radrL,WREG
				sublw	0xBC-.1					; > (LOW(700) -1) ?
				btfss	CARRY
				goto	chk_alg13				; jah, DS2438 kivi jutud k�ivad !

chk_alg8b:		movff	m_radrL,WREG
				sublw	0x8A-.1					; > (LOW(650) -1) ?
				btfss	CARRY
				goto	chk_alg9				; eip, tegeletakse DS1820-andurite ID-dega. Need on r/o ! 
				movff	m_radrL,serpartmp
				movlw	.40						; teisenda anduri aadress: high = 0, low -41
				subwf	serpartmp,W
				movff	WREG,m_radrL
				movlw	.0
				movff	WREG,m_radrH
;				clrf	m_radrH					
				bsf		dallas
				goto	chk_alg1				; jah, need on r/0 registrid !
				
chk_alg9:		movff	m_radrL,serpartmp
				movlw	.81						; teisenda anduri ID aadress: high = 0, low -81
				subwf	serpartmp,W
				movff	WREG,m_radrL
				movlw	.0
				movff	WREG,m_radrH
				bsf		dallas
				goto	chk_alg1				; jah, need on r/0 registrid !
; pwm-i registrid
chk_alg11:		movff	m_radrL,serpartmp
				movlw	.100
				subwf	serpartmp,W
				movff	WREG,m_radrL
				movff	m_radrL,serpartmp				
				LFSR	.0,pwm0set				; kirjutame nii set kui ka t��registrisse !
				LFSR	.1,pwm0work
				bcf		CARRY
				rlcf	serpartmp,W				; liidame puhvri alguse (aadress *2)
				addwf	FSR0L,F
				btfsc	CARRY	
				incf	FSR0H,F					; loetava registri aadress olemas !
				bcf		CARRY
				rlcf	serpartmp,W				; liidame puhvri alguse (aadress *2)
				addwf	FSR1L,F
				btfsc	CARRY	
				incf	FSR1H,F					; t��registri aadress samuti olemas !
				bsf		pwm
				bsf		sync					; et vaja PWMi syncida
				goto	chk_algok
; pwm/pll konfi register
chk_alg12:		movlw	.116
				movff	WREG,m_radrL
				bsf		pwm
				LFSR	.0,Register150+.0
				bsf		writeit					; oli konfidaata.kirjutada EEPROMi !
				bsf		res_pwm					; kirjutati reg-sse 150, tee PWMile reset
				goto	chk_algok
;				goto	chk_alg11
chk_alg12a:		movlw	.117
				movff	WREG,m_radrL
				bsf		pwm
				LFSR	.0,Register151+.0
;				bsf		writeit					; oli konfidaata.kirjutada EEPROMi !
				bsf		readonly				; R151 on n��d r/o !
				goto	chk_algok
;				goto	chk_alg11
chk_alg12b:	;	movlw	.116
			;	movff	WREG,m_radrL
				bsf		pwm
				bsf		res_pwm					; kirjutati reg-sse 149, tee PWMile reset
				LFSR	.0,Register149out+.0;Register149+.0
			btfsc	write
			LFSR	.0,Register149in+.0;Register149+.0
				goto	chk_algok

chk_alg13:		movff	m_radrH,WREG
				sublw	.3
				btfsc	ZERO
				goto	chk_alg14				; DS2438 kivi ID-de registrite p��rdumine
				movff	m_radrL,WREG
				sublw	0xEE-.1					; > (LOW(750) -1) ?
				btfss	CARRY
				goto	chk_alg14				; DS2438 kivi ID-de registrite p��rdumine
; DS2438 n�idud
				LFSR	.0,DS2438_1				; n�itude registrite baas
				movlw	.0
				movff	WREG,m_radrH
				movff	m_radrL,WREG
				sublw	LOW(.700)				; aadress-0xBC
				bcf		CARRY
				movff	WREG,m_radrL
				movwf	serpartmp
				rlcf	serpartmp,F				; *2
				addwf	FSR0L,F
				btfsc	CARRY	
				incf	FSR0H,F					; loetava registri aadress olemas !
				bsf		pwm
				goto	chk_alg1				; read-only !
; DS2438 ID-d (750...)
chk_alg14:		LFSR	.0,DS24381wa			; ID-de registrite baas
        		movlw	HIGH(.750)				; aadress-0x2EE (750 dec.)
				movwf	PRODL
				movff	m_radrL,serpartmp
				movlw	LOW(.750)
				subwf	serpartmp,F;W
				movff	serpartmp,m_radrL
				movff	m_radrH,serpartmp
				movff	m_radrH,WREG
				btfss	CARRY
				incfsz	PRODL,W
				subwf	serpartmp,F;W
				movff	serpartmp,m_radrH
				movff	m_radrL,serpartmp
				bcf		CARRY					; *2
				rlcf	serpartmp,W
				addwf	FSR0L,F
				btfsc	CARRY	
				incf	FSR0H,F					; loetava registri aadress olemas !
				bsf		pwm
				goto	chk_alg1				; read-only !
chk_algok:		bcf		CARRY					; algusaadress oli lubatud piirides => Cy=0
				return
chk_algbad:		bsf		CARRY					; algusaadress oli �le piiri => Cy=1
				bcf		clrsticky	
				return
;===============================================================================
;chk_len:		movff	n_regH,WREG				; loetavate registrite arv (HIGH)
;				addlw	.0
;				btfss	ZERO
;				goto	chk_lenbad				; liiga pikk lugemise/kirjutamise soov -> per...
;				movff	n_regL,WREG				; loetavate registrite arv (LOW)
;				movff	m_radrL,serpartmp
;				addwf	serpartmp,W				; mitut registrit sooviti t��delda ?
;				sublw	maxregisters
;				btfss	CARRY
;				goto	chk_lenbad
;
;				movff	n_regL,serpartmp
;				rlcf	serpartmp,W				; ei tohi puhvri piiridest v�lja minna ~!~~
;				addlw	.5
;				sublw	RSBufLen
;				btfss	CARRY
;				goto	chk_lenbad
;
;				bsf		clrsticky				; oletame, et lubataxe sticky't maha v�tta
;
;				btfss	reg0					; kas vaid 1 register & reg0 ? siis clrsticky =0 !!!
;				goto	chk_len1				; ei, siis k�ik kenasti
;				movff	n_regL,WREG				; loetavate registrite arv (LOW)
;				sublw	.1
;				btfss	ZERO
;				goto	chk_len1				; ei, siis k�ik kenasti
;				bcf		clrsticky				; jah, siis oli vaid reg. 0 lugemine => ei luba sticky't maha v�tta
;chk_len1:		bcf		CARRY					; l�ppaadress oli lubatud piirides => Cy=0
;				return
;chk_lenbad:		bsf		CARRY					; l�ppaadress oli �le piiri => Cy=1
;				bcf		clrsticky	
;				return
;;===============================================================================
valekask:		movlw	IllFunc					; vale k�sk, mine per...
				goto	send_err
valedata:		movlw	IllDAdr
send_err:		movff	WREG,Puhver+.2			; vea kood puhvrisse
				LFSR	.1,Puhver				; formeerime vastuse saatepuhvrisse
				movlw	0xFF					
				movwf	_RS485chkH				; valmistume kontrollsumma arvutamiseks	
				movwf	_RS485chk
				clrf	bytecnt
				movf	Register274+.1,W
				movwf	POSTINC1				; oma aadress
				call	mb_crc16
				incf	bytecnt,F				; loendame baite
			movff	Puhver+.1,WREG
			addlw	0x80
			movff	WREG,Puhver+.1
;				bsf		Puhver+.1,.7
				movf	POSTINC1,W				; bump'i pointerit
				movff	Puhver+.1,WREG
				call	mb_crc16
				incf	bytecnt,F				; loendame baite
				movf	POSTINC1,W				; vea kood siin juba kirjas, arvutame CRC-sse ja loendame ikkagi
				call	mb_crc16
				incf	bytecnt,F				; loendame baite
				goto	paketilopp				; summa puhvisse ja saada teele
;===============================================================================
reset_ser2:;	bsf	Dout0
			goto	reset_ser
reset_ser1:		call	reload_side				; sidetaimeri reload
reset_ser:		clrf	bytecnt					; - baitide loendi
				bcf		SerialTimeOut			
				bcf		SerialTimerOn			; taimer seisma
				movlw	serialtime
				movwf	serialtimer
				banksel	RCSTA1
				bcf		RCSTA1,CREN
				bsf		RCSTA1,CREN
				banksel	RCREG1
				movf	RCREG1,W
				banksel	.0
				movlw	.8						; ootame 8 baidist paketti
				movwf	countL
				bcf		cmd10
				bcf		cmd_ok

				bcf		Dir						; lisa Dir signaal nulli
				bcf		PIR3,CCP2IF
				bsf		PIE3,CCP2IE				; ja lubame j�lle sellise tekiamise

				return
RREnd1:			movlw	serialtime				; relae ootetaimer
				movwf	serialtimer
;				call	reload_side				; sidetaimeri reload
				return
;===============================================================================
; ********************** funktsioonid ******************************************
;===============================================================================
inc_count:		bcf		CARRY
				movf	INDF0,W
				addlw	.1
				movwf	INDF0					; loendame 1 hoolega �ra debouncetud pulsi juurde
				btfss	CARRY
				goto	inc_end
				decf	FSR0L,F
				bcf		CARRY
				movf	INDF0,W
				addlw	.1
				movwf	INDF0				
				btfss	CARRY
				goto	inc_end
				decf	FSR0L,F
				bcf		CARRY
				movf	INDF0,W
				addlw	.1
				movwf	INDF0				
				btfss	CARRY
				goto	inc_end
				decf	FSR0L,F
				bcf		CARRY
				movf	INDF0,W
				addlw	.1
				movwf	INDF0				
inc_end:		return
;===============================================================================
;*********************** Taimer 1'e INT ****************************************
;===============================================================================
; * T1 on s�steemi taimer intervalliga 10 ms.
T1int:			bcf     PIR1,TMR1IF    			; katkestuse n�ue maha 
				movlw	T1resoL					; lae taimer 1 uuesti
				movwf	TMR1L
				movlw	T1resoH
				movwf	TMR1H
;********* Vahipeni ketti **************
				clrwdt                    		; WDT nullida 
;**** Wiegandi paketi l�pu taimerid ****
T1intwiegA:		btfss	wiegandAtmron
				goto	T1intwiegB
				decfsz	wiegandAtimer
				goto	T1intwiegB
				movlw	wiegandtime
				movwf	wiegandAtimer
				bsf		wiegandAto
				bcf		wiegandAtmron
				movff	WAbitcount,Register10+.0
				clrf	WAbitcount
				btfsc	wiegandBtmron			; kui teise lugeja lugemine pooleli, ei lase veel sidet �les v�tta
				goto	T1intwiegB
				banksel	PIE1
				bsf		PIE1,RC1IE
				banksel	.0
T1intwiegB:		btfss	wiegandBtmron
				goto	T1intnw
				decfsz	wiegandBtimer
				goto	T1intnw
				movlw	wiegandtime
				movwf	wiegandBtimer
				bsf		wiegandBto
				bcf		wiegandBtmron
				movff	WBbitcount,Register10+.1
				clrf	WBbitcount
				btfsc	wiegandAtmron			; kui teise lugeja lugemine pooleli, ei lase veel sidet �les v�tta
				goto	T1intnw
				banksel	PIE1
				bsf		PIE1,RC1IE
				banksel	.0
;**** sekundi aja taimerid ****
T1intnw:		decfsz	sekunditmr
				goto	T1int_1
				movlw	sekundiaeg
				movwf	sekunditmr
				call	GetTemp
;**** uptime loendamine ****
T1_uptime:		LFSR	.0,LoendiUP+.3
				call	inc_count				; tixub 1 sekundi edukat uptime't juurde
;**** uptime loendamine ****		
				btfss	sidetmron				; kui 30s jooksul meiega ei suheldud, v�tab default sideparameetrid
				goto	T1int_0
				decfsz	sidetaimer
				goto	T1int_0
				call	setup_serial0			; v�tab vaikimisi seriali seaded
				call	reset_ser1				; sidetaimeri reload
				bcf		sidetmron				; sidetaimer seisma
;---------- on seda vaja ? -------------
				bcf		PIR1,RC1IF
				banksel	PIE1
				bsf		PIE1,RC1IE				; lubame uue k�su vastuv�ttu kui miskip�rast oli keelatud
				banksel	.0
;---------- on seda vaja ? -------------
;**** Resettide taimerid ****
T1int_0:		call	decrtmrs
;**** sidepaketi taimer ****
T1int_1:		btfss	SerialTimerOn			; seriali taimer k�ib?
				goto	T1int_2
				decf	serialtimer,F			; aeg t�is?
				movf	serialtimer,W
				addlw	.0
				btfss	ZERO
				goto	T1int_2
				call	reset_ser
				bcf		PIR1,RC1IF				; katkestuse n�ue maha
; *** loendite sisendite debounce ***
T1int_2:		btfss	sens1tmron				; loendi 1: debounceme ?
				goto	T1int_3					; eip
				decfsz	Loendi1tmr
				goto	T1int_3
				movlw	senstime				; on aeg !
				movwf	Loendi1tmr
				bcf		sens1tmron
				LFSR	.0,Loendi1+.3
				call	inc_count				; tixub 1 pulsi 
T1int_3:		btfss	sens2tmron				; loendi 2
				goto	T1int_4					
				decfsz	Loendi2tmr
				goto	T1int_4
				movlw	senstime			
				movwf	Loendi2tmr
				bcf		sens2tmron
				LFSR	.0,Loendi2+.3
				call	inc_count			
T1int_4:		btfss	sens3tmron				; loendi 3
				goto	T1int_5				
				decfsz	Loendi3tmr
				goto	T1int_5
				movlw	senstime				
				movwf	Loendi3tmr
				bcf		sens3tmron
				LFSR	.0,Loendi3+.3
				call	inc_count				
T1int_5:		btfss	sens4tmron				; loendi 4
				goto	T1int_6				
				decfsz	Loendi4tmr
				goto	T1int_6
				movlw	senstime				
				movwf	Loendi4tmr
				bcf		sens4tmron
				LFSR	.0,Loendi4+.3
				call	inc_count				
T1int_6:		btfss	sens5tmron				; loendi 5
				goto	T1int_7				
				decfsz	Loendi5tmr
				goto	T1int_7
				movlw	senstime				
				movwf	Loendi5tmr
				bcf		sens5tmron
				LFSR	.0,Loendi5+.3
				call	inc_count				
T1int_7:		btfss	sens6tmron				; loendi 6
				goto	T1int_8				
				decfsz	Loendi6tmr
				goto	T1int_8
				movlw	senstime				
				movwf	Loendi6tmr
				bcf		sens6tmron
				LFSR	.0,Loendi6+.3
				call	inc_count				
T1int_8:		btfss	sens7tmron				; loendi 7
				goto	T1int_9				
				decfsz	Loendi7tmr
				goto	T1int_9
				movlw	senstime				
				movwf	Loendi7tmr
				bcf		sens7tmron
				LFSR	.0,Loendi7+.3
				call	inc_count				
T1int_9:		btfss	sens8tmron				; loendi 8
				goto	Din_0				
				decfsz	Loendi8tmr
				goto	Din_0
				movlw	senstime				
				movwf	Loendi8tmr
				bcf		sens8tmron
				LFSR	.0,Loendi8+.3
				call	inc_count				
;**** sisendite debounce ***** - > sisend 0 (DIN plokist)
Din_0:			btfss	in_sticky
				goto	Din_0a
				btfsc	dinstuck0
				goto	Din_1
Din_0a:			btfsc	Din0					; sisend 0 madal ?
				goto	Din0high				; ei, k�rge, seedi seda
				btfss	in_debounce				; kas debounce't teeb ?
				goto	Din_0stk				; ei, arvestab kohe
				btfss	d0timeron				; jah, kas juba teame ?
				goto	Din0high1				; ei, k�ivita deb. timer kui vaja
				decfsz	din0tmr					; debouncetud?
				goto	Din_1					; ei veel, v�ta j�rgmine sisend
Din_0stk:		bsf		dinpress0				; jah, v�tame arvesse
				bsf		dinstuck0				; blokeerime biti igal juhul
Din_0setlow:;	bsf		pank1
				bsf		Register1,.0
;				bcf		pank1
Din0_lstckl:	bcf		d0timeron
				goto	Din_1					; v�ta j�rgmine sisend
Din0high:		btfss	dinpress0				; oli k�rge enne ?
				goto	Din_1					; ei, v�ta j�rgmine sisend
				btfss	in_debounce				; kas debounce't teeb ?
				goto	Din_0stkh				; ei, arvestab kohe
				btfss	d0timeron				; lahti laskmise debounce k�ib?
				goto	Din0strt				; ei, paneme k�ima
				decfsz	din0tmr
				goto	Din_1					; v�ta j�rgmine sisend
				bcf		dinpress0				; loeme lahti lastuks
Din_0stkh:		bsf		dinstuck0				; blokeerime biti igal juhul
Din_0sethi:		bcf		Register1,.0
Din0_lstckh:	bcf		d0timeron
				goto	Din_1					; v�ta j�rgmine sisend
Din0high1:		btfsc	dinpress0
				goto	Din_1					; v�ta j�rgmine sisend
Din0strt:		movlw	debtime					; k�ivitame debounce taimeri
				movwf	din0tmr
				bsf		d0timeron
;**** sisendite debounce ***** - > sisend 1 (DIN plokist)
Din_1:			btfss	in_sticky
				goto	Din_1a
				btfsc	dinstuck1
				goto	Din_2
Din_1a:			btfsc	Din1					; sisend 1 madal ?
				goto	Din1high				; ei, k�rge, seedi seda
				btfss	in_debounce				; kas debounce't teeb ?
				goto	Din_1stk				; ei, arvestab kohe
				btfss	d1timeron				; jah, kas juba teame ?
				goto	Din1high1				; ei, k�ivita deb. timer kui vaja
				decfsz	din1tmr					; debouncetud?
				goto	Din_2					; ei veel, v�ta j�rgmine sisend
Din_1stk:		bsf		dinpress1				; jah, v�tame arvesse
				bsf		dinstuck1				; blokeerime biti igal juhul
Din_1setlow:	bsf		Register1,.1
Din1_lstckl:	bcf		d1timeron
				goto	Din_2					; v�ta j�rgmine sisend
Din1high:		btfss	dinpress1				; oli k�rge enne ?
				goto	Din_2					; ei, v�ta j�rgmine sisend
				btfss	in_debounce				; kas debounce't teeb ?
				goto	Din_1stkh				; ei, arvestab kohe
				btfss	d1timeron				; lahti laskmise debounce k�ib?
				goto	Din1strt				; ei, paneme k�ima
				decfsz	din1tmr
				goto	Din_2					; v�ta j�rgmine sisend
				bcf		dinpress1				; loeme lahti lastuks
Din_1stkh:		bsf		dinstuck1				; blokeerime biti igal juhul
Din_1sethi:		bcf		Register1,.1
Din1_lstckh:	bcf		d1timeron
				goto	Din_2					; v�ta j�rgmine sisend
Din1high1:		btfsc	dinpress1
				goto	Din_2					; v�ta j�rgmine sisend
Din1strt:		movlw	debtime					; k�ivitame debounce taimeri
				movwf	din1tmr
				bsf		d1timeron
;**** sisendite debounce ***** - > sisend 2 (DIN plokist)
Din_2:			btfss	in_sticky
				goto	Din_2a
				btfsc	dinstuck2
				goto	Din_3
Din_2a:			btfsc	Din2					; sisend 2 madal ?
				goto	Din2high				; ei, k�rge, seedi seda
				btfss	in_debounce				; kas debounce't teeb ?
				goto	Din_2stk				; ei, arvestab kohe
				btfss	d2timeron				; jah, kas juba teame ?
				goto	Din2high1				; ei, k�ivita deb. timer kui vaja
				decfsz	din2tmr					; debouncetud?
				goto	Din_3					; ei veel, v�ta j�rgmine sisend
Din_2stk:		bsf		dinpress2				; jah, v�tame arvesse
				bsf		dinstuck2				; blokeerime biti igal juhul
Din_2setlow:;	bsf		pank1
				bsf		Register1,.2
;				bcf		pank1
Din2_lstckl:	bcf		d2timeron
				goto	Din_3					; v�ta j�rgmine sisend
Din2high:		btfss	dinpress2				; oli k�rge enne ?
				goto	Din_3					; ei, v�ta j�rgmine sisend
				btfss	in_debounce				; kas debounce't teeb ?
				goto	Din_2stkh				; ei, arvestab kohe
				btfss	d2timeron				; lahti laskmise debounce k�ib?
				goto	Din2strt				; ei, paneme k�ima
				decfsz	din2tmr
				goto	Din_3					; v�ta j�rgmine sisend
				bcf		dinpress2				; loeme lahti lastuks
Din_2stkh:		bsf		dinstuck2				; blokeerime biti igal juhul
Din_2sethi:;		bsf		pank1
				bcf		Register1,.2
;				bcf		pank1
Din2_lstckh:	bcf		d2timeron
				goto	Din_3					; v�ta j�rgmine sisend
Din2high1:		btfsc	dinpress2
				goto	Din_3					; v�ta j�rgmine sisend
Din2strt:		movlw	debtime					; k�ivitame debounce taimeri
				movwf	din2tmr
				bsf		d2timeron
;**** sisendite debounce ***** - > sisend 3 (DIN plokist)
Din_3:			btfss	in_sticky
				goto	Din_3a
				btfsc	dinstuck3
				goto	Din_4
Din_3a:			btfsc	Din3					; sisend 3 madal ?
				goto	Din3high				; ei, k�rge, seedi seda
				btfss	in_debounce				; kas debounce't teeb ?
				goto	Din_3stk				; ei, arvestab kohe
				btfss	d3timeron				; jah, kas juba teame ?
				goto	Din3high1				; ei, k�ivita deb. timer kui vaja
				decfsz	din3tmr					; debouncetud?
				goto	Din_4					; ei veel, v�ta j�rgmine sisend
Din_3stk:		bsf		dinpress3				; jah, v�tame arvesse
;				btfss	in_sticky				; sticky bitti arvestame ?
;				goto	Din_3setlow				; eip
;				btfsc	dinstuck3				; kas muutus veel teavitamata?
;				goto	Din3_lstckl				; teavitus saatmata, bitti ei muuda
				bsf		dinstuck3				; blokeerime biti igal juhul
Din_3setlow:;	bsf		pank1
				bsf		Register1,.3
;				bcf		pank1
Din3_lstckl:	bcf		d3timeron
				goto	Din_4					; v�ta j�rgmine sisend
Din3high:		btfss	dinpress3				; oli k�rge enne ?
				goto	Din_4					; ei, v�ta j�rgmine sisend
				btfss	in_debounce				; kas debounce't teeb ?
				goto	Din_3stkh				; ei, arvestab kohe
				btfss	d3timeron				; lahti laskmise debounce k�ib?
				goto	Din3strt				; ei, paneme k�ima
				decfsz	din3tmr
				goto	Din_4					; v�ta j�rgmine sisend
				bcf		dinpress3				; loeme lahti lastuks
Din_3stkh:;		btfss	in_sticky				; sticky bitti arvestame ?
;				goto	Din_3sethi				; eip
;				btfsc	dinstuck3				; kas muutus veel teavitamata?
;				goto	Din3_lstckh				; teavitus saatmata, bitti ei muuda
				bsf		dinstuck3				; blokeerime biti igal juhul
Din_3sethi:;		bsf		pank1
				bcf		Register1,.3
;				bcf		pank1
Din3_lstckh:	bcf		d3timeron
				goto	Din_4					; v�ta j�rgmine sisend
Din3high1:		btfsc	dinpress3
				goto	Din_4					; v�ta j�rgmine sisend
Din3strt:		movlw	debtime					; k�ivitame debounce taimeri
				movwf	din3tmr
				bsf		d3timeron
;				goto	Din_4					; v�ta j�rgmine sisend
;**** sisendite debounce ***** - > sisend 4 (DIN plokist)
Din_4:			btfss	in_sticky
				goto	Din_4a
				btfsc	dinstuck4
				goto	Din_5
Din_4a:			btfsc	Din4					; sisend 4 madal ?
				goto	Din4high				; ei, k�rge, seedi seda
				btfss	in_debounce				; kas debounce't teeb ?
				goto	Din_4stk				; ei, arvestab kohe
				btfss	d4timeron				; jah, kas juba teame ?
				goto	Din4high1				; ei, k�ivita deb. timer kui vaja
				decfsz	din4tmr					; debouncetud?
				goto	Din_5					; ei veel, v�ta j�rgmine sisend
Din_4stk:		bsf		dinpress4				; jah, v�tame arvesse
;				btfss	in_sticky				; sticky bitti arvestame ?
;				goto	Din_4setlow				; eip
;				btfsc	dinstuck4				; kas muutus veel teavitamata?
;				goto	Din4_lstckl				; teavitus saatmata, bitti ei muuda
				bsf		dinstuck4				; blokeerime biti igal juhul
Din_4setlow:;	bsf		pank1
				bsf		Register1,.4
;				bcf		pank1
Din4_lstckl:	bcf		d4timeron
				goto	Din_5					; v�ta j�rgmine sisend
Din4high:		btfss	dinpress4				; oli k�rge enne ?
				goto	Din_5					; ei, v�ta j�rgmine sisend
				btfss	in_debounce				; kas debounce't teeb ?
				goto	Din_4stkh				; ei, arvestab kohe
				btfss	d4timeron				; lahti laskmise debounce k�ib?
				goto	Din4strt				; ei, paneme k�ima
				decfsz	din4tmr
				goto	Din_5					; v�ta j�rgmine sisend
				bcf		dinpress4				; loeme lahti lastuks
Din_4stkh:;		btfss	in_sticky				; sticky bitti arvestame ?
;				goto	Din_4sethi				; eip
;				btfsc	dinstuck4				; kas muutus veel teavitamata?
;				goto	Din4_lstckh				; teavitus saatmata, bitti ei muuda
				bsf		dinstuck4				; blokeerime biti igal juhul
Din_4sethi:;		bsf		pank1
				bcf		Register1,.4
;				bcf		pank1
Din4_lstckh:	bcf		d4timeron
				goto	Din_5					; v�ta j�rgmine sisend
Din4high1:		btfsc	dinpress4
				goto	Din_5					; v�ta j�rgmine sisend
Din4strt:		movlw	debtime					; k�ivitame debounce taimeri
				movwf	din4tmr
				bsf		d4timeron
;				goto	Din_5					; v�ta j�rgmine sisend
;**** sisendite debounce ***** - > sisend 5 (DIN plokist)
Din_5:			btfss	in_sticky
				goto	Din_5a
				btfsc	dinstuck5
				goto	Din_6
Din_5a:			btfsc	Din5					; sisend 5 madal ?
				goto	Din5high				; ei, k�rge, seedi seda
				btfss	in_debounce				; kas debounce't teeb ?
				goto	Din_5stk				; ei, arvestab kohe
				btfss	d5timeron				; jah, kas juba teame ?
				goto	Din5high1				; ei, k�ivita deb. timer kui vaja
				decfsz	din5tmr					; debouncetud?
				goto	Din_6					; ei veel, v�ta j�rgmine sisend
Din_5stk:		bsf		dinpress5				; jah, v�tame arvesse
;				btfss	in_sticky				; sticky bitti arvestame ?
;				goto	Din_5setlow				; eip
;				btfsc	dinstuck5				; kas muutus veel teavitamata?
;				goto	Din5_lstckl				; teavitus saatmata, bitti ei muuda
				bsf		dinstuck5				; blokeerime biti igal juhul
Din_5setlow:;	bsf		pank1
				bsf		Register1,.5
;				bcf		pank1
Din5_lstckl:	bcf		d5timeron
				goto	Din_6					; v�ta j�rgmine sisend
Din5high:		btfss	dinpress5				; oli k�rge enne ?
				goto	Din_6					; ei, v�ta j�rgmine sisend
				btfss	in_debounce				; kas debounce't teeb ?
				goto	Din_5stkh				; ei, arvestab kohe
				btfss	d5timeron				; lahti laskmise debounce k�ib?
				goto	Din5strt				; ei, paneme k�ima
				decfsz	din5tmr
				goto	Din_6					; v�ta j�rgmine sisend
				bcf		dinpress5				; loeme lahti lastuks
Din_5stkh:;		btfss	in_sticky				; sticky bitti arvestame ?
;				goto	Din_5sethi				; eip
;				btfsc	dinstuck5				; kas muutus veel teavitamata?
;				goto	Din5_lstckh				; teavitus saatmata, bitti ei muuda
				bsf		dinstuck5				; blokeerime biti igal juhul
Din_5sethi:;		bsf		pank1
				bcf		Register1,.5
;				bcf		pank1
Din5_lstckh:	bcf		d5timeron
				goto	Din_6					; v�ta j�rgmine sisend
Din5high1:		btfsc	dinpress5
				goto	Din_6					; v�ta j�rgmine sisend
Din5strt:		movlw	debtime					; k�ivitame debounce taimeri
				movwf	din5tmr
				bsf		d5timeron
;				goto	Din_6					; v�ta j�rgmine sisend
;**** sisendite debounce ***** - > sisend 6 (DIN plokist)
Din_6:			btfss	in_sticky
				goto	Din_6a
				btfsc	dinstuck6
				goto	Din_7
Din_6a:			btfsc	Din6					; sisend 6 madal ?
				goto	Din6high				; ei, k�rge, seedi seda
				btfss	in_debounce				; kas debounce't teeb ?
				goto	Din_6stk				; ei, arvestab kohe
				btfss	d6timeron				; jah, kas juba teame ?
				goto	Din6high1				; ei, k�ivita deb. timer kui vaja
				decfsz	din6tmr					; debouncetud?
				goto	Din_7					; ei veel, v�ta j�rgmine sisend
Din_6stk:		bsf		dinpress6				; jah, v�tame arvesse
;				btfss	in_sticky				; sticky bitti arvestame ?
;				goto	Din_6setlow				; eip
;				btfsc	dinstuck6				; kas muutus veel teavitamata?
;				goto	Din6_lstckl				; teavitus saatmata, bitti ei muuda
				bsf		dinstuck6				; blokeerime biti igal juhul
Din_6setlow:;	bsf		pank1
				bsf		Register1,.6
;				bcf		pank1
Din6_lstckl:	bcf		d6timeron
				goto	Din_7					; v�ta j�rgmine sisend
Din6high:		btfss	dinpress6				; oli k�rge enne ?
				goto	Din_7					; ei, v�ta j�rgmine sisend
				btfss	in_debounce				; kas debounce't teeb ?
				goto	Din_6stkh				; ei, arvestab kohe
				btfss	d6timeron				; lahti laskmise debounce k�ib?
				goto	Din6strt				; ei, paneme k�ima
				decfsz	din6tmr
				goto	Din_7					; v�ta j�rgmine sisend
				bcf		dinpress6				; loeme lahti lastuks
Din_6stkh:	;	btfss	in_sticky				; sticky bitti arvestame ?
;				goto	Din_6sethi				; eip
;				btfsc	dinstuck6				; kas muutus veel teavitamata?
;				goto	Din6_lstckh				; teavitus saatmata, bitti ei muuda
				bsf		dinstuck6				; blokeerime biti igal juhul
Din_6sethi:;		bsf		pank1
				bcf		Register1,.6
;				bcf		pank1
Din6_lstckh:	bcf		d6timeron
				goto	Din_7					; v�ta j�rgmine sisend
Din6high1:		btfsc	dinpress6
				goto	Din_7					; v�ta j�rgmine sisend
Din6strt:		movlw	debtime					; k�ivitame debounce taimeri
				movwf	din6tmr
				bsf		d6timeron
;				goto	Din_7					; v�ta j�rgmine sisend
;**** sisendite debounce ***** - > sisend 7 (DIN plokist)
Din_7:			btfss	in_sticky
				goto	Din_7a
				btfsc	dinstuck7
				goto	Din_8
Din_7a:			btfsc	Din7					; sisend 7 madal ?
				goto	Din7high				; ei, k�rge, seedi seda
				btfss	in_debounce				; kas debounce't teeb ?
				goto	Din_7stk				; ei, arvestab kohe
				btfss	d7timeron				; jah, kas juba teame ?
				goto	Din7high1				; ei, k�ivita deb. timer kui vaja
				decfsz	din7tmr					; debouncetud?
				goto	Din_8					; ei veel, v�ta j�rgmine sisend
Din_7stk:		bsf		dinpress7				; jah, v�tame arvesse
;				btfss	in_sticky				; sticky bitti arvestame ?
;				goto	Din_7setlow				; eip
;				btfsc	dinstuck7				; kas muutus veel teavitamata?
;				goto	Din7_lstckl				; teavitus saatmata, bitti ei muuda
				bsf		dinstuck7				; blokeerime biti igal juhul
Din_7setlow:;	bsf		pank1
				bsf		Register1,.7
;				bcf		pank1
Din7_lstckl:	bcf		d7timeron
				goto	Din_8					; v�ta j�rgmine sisend
Din7high:		btfss	dinpress7				; oli k�rge enne ?
				goto	Din_8					; ei, v�ta j�rgmine sisend
				btfss	in_debounce				; kas debounce't teeb ?
				goto	Din_7stkh				; ei, arvestab kohe
				btfss	d7timeron				; lahti laskmise debounce k�ib?
				goto	Din7strt				; ei, paneme k�ima
				decfsz	din7tmr
				goto	Din_8					; v�ta j�rgmine sisend
				bcf		dinpress7				; loeme lahti lastuks
Din_7stkh:;		btfss	in_sticky				; sticky bitti arvestame ?
;				goto	Din_7sethi				; eip
;				btfsc	dinstuck7				; kas muutus veel teavitamata?
;				goto	Din7_lstckh				; teavitus saatmata, bitti ei muuda
				bsf		dinstuck7				; blokeerime biti igal juhul
Din_7sethi:	;	bsf		pank1
				bcf		Register1,.7
;				bcf		pank1
Din7_lstckh:	bcf		d7timeron
				goto	Din_8					; v�ta j�rgmine sisend
Din7high1:		btfsc	dinpress7
				goto	Din_8					; v�ta j�rgmine sisend
Din7strt:		movlw	debtime					; k�ivitame debounce taimeri
				movwf	din7tmr
				bsf		d7timeron
;				goto	Din_8					; v�ta j�rgmine sisend
Din_8:
;___________________________
;**** sisendite debounce ***** - > sisend 0 (AIN plokist)
Ain_0:			movf	PORTA,W					; loeme ANA-pordi seisu
				andlw	0x0F
				movwf	Registertemp5
				btfsc	PORTA,.5
				bsf		Registertemp5,.4
				btfsc	PORTE,.0
				bsf		Registertemp5,.5
				btfsc	PORTE,.1
				bsf		Registertemp5,.6
				btfsc	PORTE,.2
				bsf		Registertemp5,.7
				movf	Register271+.0,W		; lisa: XORime tulemuse sellega l�bi
				xorwf	Registertemp5,F;W

				btfsc	Register275+.1,.0		; kas on sisend ?
				goto	Ain_0sethi				; eip, kirjutab nulli registrisse
				btfss	Register271+.1,.0		; kas on digisisend ?
				goto	Ain_0sethi				; eip, kirjutab nulli registrisse
				btfss	in_sticky
				goto	Ain_0a
				btfsc	ainstuck0
				goto	Ain_1
Ain_0a:			btfsc	Ana0					; sisend 0 madal ?
				goto	Ain0high				; ei, k�rge, seedi seda
				btfss	in_debounce				; kas debounce't teeb ?
				goto	Ain_0stk				; ei, arvestab kohe
				btfss	a0timeron				; jah, kas juba teame ?
				goto	Ain0high1				; ei, k�ivita deb. timer kui vaja
				decfsz	ain0tmr					; debouncetud?
				goto	Ain_1					; ei veel, v�ta j�rgmine sisend
Ain_0stk:		bsf		ainpress0				; jah, v�tame arvesse
;				btfss	in_sticky				; sticky bitti arvestame ?
;				goto	Ain_0setlow				; eip
;				btfsc	ainstuck0				; kas muutus veel teavitamata?
;				goto	Ain0_lstckl				; teavitus saatmata, bitti ei muuda
				bsf		ainstuck0				; blokeerime biti igal juhul
Ain_0setlow:;	bsf		pank1
				bsf		Register1+.1,.0
;				bcf		pank1
Ain0_lstckl:	bcf		a0timeron
				goto	Ain_1					; v�ta j�rgmine sisend
Ain0high:		btfss	ainpress0				; oli k�rge enne ?
				goto	Ain_1					; ei, v�ta j�rgmine sisend
				btfss	in_debounce				; kas debounce't teeb ?
				goto	Ain_0stkh				; ei, arvestab kohe
				btfss	a0timeron				; lahti laskmise debounce k�ib?
				goto	Ain0strt				; ei, paneme k�ima
				decfsz	ain0tmr
				goto	Ain_1					; v�ta j�rgmine sisend
				bcf		ainpress0				; loeme lahti lastuks
Ain_0stkh:;		btfss	in_sticky				; sticky bitti arvestame ?
;				goto	Ain_0sethi				; eip
;				btfsc	ainstuck0				; kas muutus veel teavitamata?
;				goto	Ain0_lstckh				; teavitus saatmata, bitti ei muuda
				bsf		ainstuck0				; blokeerime biti igal juhul
Ain_0sethi:;		bsf		pank1
				bcf		Register1+.1,.0
;				bcf		pank1
Ain0_lstckh:	bcf		a0timeron
				goto	Ain_1					; v�ta j�rgmine sisend
Ain0high1:		btfsc	ainpress0
				goto	Ain_1					; v�ta j�rgmine sisend
Ain0strt:		movlw	debtime					; k�ivitame debounce taimeri
				movwf	ain0tmr
				bsf		a0timeron
;				goto	Ain_1					; v�ta j�rgmine sisend
;**** sisendite debounce ***** - > sisend 1 (AIN plokist)
Ain_1:			btfsc	Register275+.1,.1		; kas on sisend ?
				goto	Ain_1sethi				; eip, kirjutab nulli registrisse
				btfss	Register271+.1,.1		; kas on digisisend ?
				goto	Ain_1sethi				; eip, kirjutab nulli registrisse
				btfss	in_sticky
				goto	Ain_1a
				btfsc	ainstuck1
				goto	Ain_2
Ain_1a:			btfsc	Ana1					; sisend 1 madal ?
				goto	Ain1high				; ei, k�rge, seedi seda
				btfss	in_debounce				; kas debounce't teeb ?
				goto	Ain_1stk				; ei, arvestab kohe
				btfss	a1timeron				; jah, kas juba teame ?
				goto	Ain1high1				; ei, k�ivita deb. timer kui vaja
				decfsz	ain1tmr					; debouncetud?
				goto	Ain_2					; ei veel, v�ta j�rgmine sisend
Ain_1stk:		bsf		ainpress1				; jah, v�tame arvesse
;				btfss	in_sticky				; sticky bitti arvestame ?
;				goto	Ain_1setlow				; eip
;				btfsc	ainstuck1				; kas muutus veel teavitamata?
;				goto	Ain1_lstckl				; teavitus saatmata, bitti ei muuda
				bsf		ainstuck1				; blokeerime biti igal juhul
Ain_1setlow:;	bsf		pank1
				bsf		Register1+.1,.1
;				bcf		pank1
Ain1_lstckl:	bcf		a1timeron
				goto	Ain_2					; v�ta j�rgmine sisend
Ain1high:		btfss	ainpress1				; oli k�rge enne ?
				goto	Ain_2					; ei, v�ta j�rgmine sisend
				btfss	in_debounce				; kas debounce't teeb ?
				goto	Ain_1stkh				; ei, arvestab kohe
				btfss	a1timeron				; lahti laskmise debounce k�ib?
				goto	Ain1strt				; ei, paneme k�ima
				decfsz	ain1tmr
				goto	Ain_2					; v�ta j�rgmine sisend
				bcf		ainpress1				; loeme lahti lastuks
Ain_1stkh:;		btfss	in_sticky				; sticky bitti arvestame ?
;				goto	Ain_1sethi				; eip
;				btfsc	ainstuck1				; kas muutus veel teavitamata?
;				goto	Ain1_lstckh				; teavitus saatmata, bitti ei muuda
				bsf		ainstuck1				; blokeerime biti igal juhul
Ain_1sethi:;		bsf		pank1
				bcf		Register1+.1,.1
;				bcf		pank1
Ain1_lstckh:	bcf		a1timeron
				goto	Ain_2					; v�ta j�rgmine sisend
Ain1high1:		btfsc	ainpress1
				goto	Ain_2					; v�ta j�rgmine sisend
Ain1strt:		movlw	debtime					; k�ivitame debounce taimeri
				movwf	ain1tmr
				bsf		a1timeron
;				goto	Ain_2					; v�ta j�rgmine sisend
;**** sisendite debounce ***** - > sisend 2 (AIN plokist)
Ain_2:			btfsc	Register275+.1,.2		; kas on sisend ?
				goto	Ain_2sethi				; eip, kirjutab nulli registrisse
				btfss	Register271+.1,.2		; kas on digisisend ?
				goto	Ain_2sethi				; eip, kirjutab nulli registrisse
				btfss	in_sticky
				goto	Ain_2a
				btfsc	ainstuck2
				goto	Ain_3
Ain_2a:			btfsc	Ana2					; sisend 2 madal ?
				goto	Ain2high				; ei, k�rge, seedi seda
				btfss	in_debounce				; kas debounce't teeb ?
				goto	Ain_2stk				; ei, arvestab kohe
				btfss	a2timeron				; jah, kas juba teame ?
				goto	Ain2high1				; ei, k�ivita deb. timer kui vaja
				decfsz	ain2tmr					; debouncetud?
				goto	Ain_3					; ei veel, v�ta j�rgmine sisend
Ain_2stk:		bsf		ainpress2				; jah, v�tame arvesse
;				btfss	in_sticky				; sticky bitti arvestame ?
;				goto	Ain_2setlow				; eip
;				btfsc	ainstuck2				; kas muutus veel teavitamata?
;				goto	Ain2_lstckl				; teavitus saatmata, bitti ei muuda
				bsf		ainstuck2				; blokeerime biti igal juhul
Ain_2setlow:;	bsf		pank1
				bsf		Register1+.1,.2
;				bcf		pank1
Ain2_lstckl:	bcf		a2timeron
				goto	Ain_3					; v�ta j�rgmine sisend
Ain2high:		btfss	ainpress2				; oli k�rge enne ?
				goto	Ain_3					; ei, v�ta j�rgmine sisend
				btfss	in_debounce				; kas debounce't teeb ?
				goto	Ain_2stkh				; ei, arvestab kohe
				btfss	a2timeron				; lahti laskmise debounce k�ib?
				goto	Ain2strt				; ei, paneme k�ima
				decfsz	ain2tmr
				goto	Ain_3					; v�ta j�rgmine sisend
				bcf		ainpress2				; loeme lahti lastuks
Ain_2stkh:;		btfss	in_sticky				; sticky bitti arvestame ?
;				goto	Ain_2sethi				; eip
;				btfsc	ainstuck2				; kas muutus veel teavitamata?
;				goto	Ain2_lstckh				; teavitus saatmata, bitti ei muuda
				bsf		ainstuck2				; blokeerime biti igal juhul
Ain_2sethi:;		bsf		pank1
				bcf		Register1+.1,.2
;				bcf		pank1
Ain2_lstckh:	bcf		a2timeron
				goto	Ain_3					; v�ta j�rgmine sisend
Ain2high1:		btfsc	ainpress2
				goto	Ain_3					; v�ta j�rgmine sisend
Ain2strt:		movlw	debtime					; k�ivitame debounce taimeri
				movwf	ain2tmr
				bsf		a2timeron
;				goto	Ain_3					; v�ta j�rgmine sisend
;**** sisendite debounce ***** - > sisend 3 (AIN plokist)
Ain_3:			btfsc	Register275+.1,.3		; kas on sisend ?
				goto	Ain_3sethi				; eip, kirjutab nulli registrisse
				btfss	Register271+.1,.3		; kas on digisisend ?
				goto	Ain_3sethi				; eip, kirjutab nulli registrisse
				btfss	in_sticky
				goto	Ain_3a
				btfsc	ainstuck3
				goto	Ain_4
Ain_3a:			btfsc	Ana3					; sisend 3 madal ?
				goto	Ain3high				; ei, k�rge, seedi seda
				btfss	in_debounce				; kas debounce't teeb ?
				goto	Ain_3stk				; ei, arvestab kohe
				btfss	a3timeron				; jah, kas juba teame ?
				goto	Ain3high1				; ei, k�ivita deb. timer kui vaja
				decfsz	ain3tmr					; debouncetud?
				goto	Ain_4					; ei veel, v�ta j�rgmine sisend
Ain_3stk:		bsf		ainpress3				; jah, v�tame arvesse
;				btfss	in_sticky				; sticky bitti arvestame ?
;				goto	Ain_3setlow				; eip
;				btfsc	ainstuck3				; kas muutus veel teavitamata?
;				goto	Ain3_lstckl				; teavitus saatmata, bitti ei muuda
				bsf		ainstuck3				; blokeerime biti igal juhul
Ain_3setlow:;	bsf		pank1
				bsf		Register1+.1,.3
;				bcf		pank1
Ain3_lstckl:	bcf		a3timeron
				goto	Ain_4					; v�ta j�rgmine sisend
Ain3high:		btfss	ainpress3				; oli k�rge enne ?
				goto	Ain_4					; ei, v�ta j�rgmine sisend
				btfss	in_debounce				; kas debounce't teeb ?
				goto	Ain_3stkh				; ei, arvestab kohe
				btfss	a3timeron				; lahti laskmise debounce k�ib?
				goto	Ain3strt				; ei, paneme k�ima
				decfsz	ain3tmr
				goto	Ain_4					; v�ta j�rgmine sisend
				bcf		ainpress3				; loeme lahti lastuks
Ain_3stkh:;		btfss	in_sticky				; sticky bitti arvestame ?
;				goto	Ain_3sethi				; eip
;				btfsc	ainstuck3				; kas muutus veel teavitamata?
;				goto	Ain3_lstckh				; teavitus saatmata, bitti ei muuda
				bsf		ainstuck3				; blokeerime biti igal juhul
Ain_3sethi:;		bsf		pank1
				bcf		Register1+.1,.3
;				bcf		pank1
Ain3_lstckh:	bcf		a3timeron
				goto	Ain_4					; v�ta j�rgmine sisend
Ain3high1:		btfsc	ainpress3
				goto	Ain_4					; v�ta j�rgmine sisend
Ain3strt:		movlw	debtime					; k�ivitame debounce taimeri
				movwf	ain3tmr
				bsf		a3timeron
;				goto	Ain_4					; v�ta j�rgmine sisend
;**** sisendite debounce ***** - > sisend 4 (AIN plokist)
Ain_4:			btfsc	Register275+.1,.4		; kas on sisend ?
				goto	Ain_4sethi				; eip, kirjutab nulli registrisse
				btfss	Register271+.1,.4		; kas on digisisend ?
				goto	Ain_4sethi				; eip, kirjutab nulli registrisse
				btfss	in_sticky
				goto	Ain_4a
				btfsc	ainstuck4
				goto	Ain_5
Ain_4a:			btfsc	Ana4					; sisend 4 madal ?
				goto	Ain4high				; ei, k�rge, seedi seda
				btfss	in_debounce				; kas debounce't teeb ?
				goto	Ain_4stk				; ei, arvestab kohe
				btfss	a4timeron				; jah, kas juba teame ?
				goto	Ain4high1				; ei, k�ivita deb. timer kui vaja
				decfsz	ain4tmr					; debouncetud?
				goto	Ain_5					; ei veel, v�ta j�rgmine sisend
Ain_4stk:		bsf		ainpress4				; jah, v�tame arvesse
;				btfss	in_sticky				; sticky bitti arvestame ?
;				goto	Ain_4setlow				; eip
;				btfsc	ainstuck4				; kas muutus veel teavitamata?
;				goto	Ain4_lstckl				; teavitus saatmata, bitti ei muuda
				bsf		ainstuck4				; blokeerime biti igal juhul
Ain_4setlow:;	bsf		pank1
				bsf		Register1+.1,.4
;				bcf		pank1
Ain4_lstckl:	bcf		a4timeron
				goto	Ain_5					; v�ta j�rgmine sisend
Ain4high:		btfss	ainpress4				; oli k�rge enne ?
				goto	Ain_5					; ei, v�ta j�rgmine sisend
				btfss	in_debounce				; kas debounce't teeb ?
				goto	Ain_4stkh				; ei, arvestab kohe
				btfss	a4timeron				; lahti laskmise debounce k�ib?
				goto	Ain4strt				; ei, paneme k�ima
				decfsz	ain4tmr
				goto	Ain_5					; v�ta j�rgmine sisend
				bcf		ainpress4				; loeme lahti lastuks
Ain_4stkh:;		btfss	in_sticky				; sticky bitti arvestame ?
;				goto	Ain_4sethi				; eip
;				btfsc	ainstuck4				; kas muutus veel teavitamata?
;				goto	Ain4_lstckh				; teavitus saatmata, bitti ei muuda
				bsf		ainstuck4				; blokeerime biti igal juhul
Ain_4sethi:;		bsf		pank1
				bcf		Register1+.1,.4
;				bcf		pank1
Ain4_lstckh:	bcf		a4timeron
				goto	Ain_5					; v�ta j�rgmine sisend
Ain4high1:		btfsc	ainpress4
				goto	Ain_5					; v�ta j�rgmine sisend
Ain4strt:		movlw	debtime					; k�ivitame debounce taimeri
				movwf	ain4tmr
				bsf		a4timeron
;				goto	Ain_5					; v�ta j�rgmine sisend
;**** sisendite debounce ***** - > sisend 5 (AIN plokist)
Ain_5:			btfsc	Register275+.1,.5		; kas on sisend ?
				goto	Ain_5sethi				; eip, kirjutab nulli registrisse
				btfss	Register271+.1,.5		; kas on digisisend ?
				goto	Ain_5sethi				; eip, kirjutab nulli registrisse
				btfss	in_sticky
				goto	Ain_5a
				btfsc	ainstuck5
				goto	Ain_6
Ain_5a:			btfsc	Ana5					; sisend 5 madal ?
				goto	Ain5high				; ei, k�rge, seedi seda
				btfss	in_debounce				; kas debounce't teeb ?
				goto	Ain_5stk				; ei, arvestab kohe
				btfss	a5timeron				; jah, kas juba teame ?
				goto	Ain5high1				; ei, k�ivita deb. timer kui vaja
				decfsz	ain5tmr					; debouncetud?
				goto	Ain_6					; ei veel, v�ta j�rgmine sisend
Ain_5stk:		bsf		ainpress5				; jah, v�tame arvesse
;				btfss	in_sticky				; sticky bitti arvestame ?
;				goto	Ain_5setlow				; eip
;				btfsc	ainstuck5				; kas muutus veel teavitamata?
;				goto	Ain5_lstckl				; teavitus saatmata, bitti ei muuda
				bsf		ainstuck5				; blokeerime biti igal juhul
Ain_5setlow:;	bsf		pank1
				bsf		Register1+.1,.5
;				bcf		pank1
Ain5_lstckl:	bcf		a5timeron
				goto	Ain_6					; v�ta j�rgmine sisend
Ain5high:		btfss	ainpress5				; oli k�rge enne ?
				goto	Ain_6					; ei, v�ta j�rgmine sisend
				btfss	in_debounce				; kas debounce't teeb ?
				goto	Ain_5stkh				; ei, arvestab kohe
				btfss	a5timeron				; lahti laskmise debounce k�ib?
				goto	Ain5strt				; ei, paneme k�ima
				decfsz	ain5tmr
				goto	Ain_6					; v�ta j�rgmine sisend
				bcf		ainpress5				; loeme lahti lastuks
Ain_5stkh:;		btfss	in_sticky				; sticky bitti arvestame ?
;				goto	Ain_5sethi				; eip
;				btfsc	ainstuck5				; kas muutus veel teavitamata?
;				goto	Ain5_lstckh				; teavitus saatmata, bitti ei muuda
				bsf		ainstuck5				; blokeerime biti igal juhul
Ain_5sethi:	;	bsf		pank1
				bcf		Register1+.1,.5
;				bcf		pank1
Ain5_lstckh:	bcf		a5timeron
				goto	Ain_6					; v�ta j�rgmine sisend
Ain5high1:		btfsc	ainpress5
				goto	Ain_6					; v�ta j�rgmine sisend
Ain5strt:		movlw	debtime					; k�ivitame debounce taimeri
				movwf	ain5tmr
				bsf		a5timeron
;				goto	Ain_6					; v�ta j�rgmine sisend
;**** sisendite debounce ***** - > sisend 6 (AIN plokist)
Ain_6:			btfsc	Register275+.1,.6		; kas on sisend ?
				goto	Ain_6sethi				; eip, kirjutab nulli registrisse
				btfss	Register271+.1,.6		; kas on digisisend ?
				goto	Ain_6sethi				; eip, kirjutab nulli registrisse
				btfss	in_sticky
				goto	Ain_6a
				btfsc	ainstuck6
				goto	Ain_7
Ain_6a:			btfsc	Ana6					; sisend 6 madal ?
				goto	Ain6high				; ei, k�rge, seedi seda
				btfss	in_debounce				; kas debounce't teeb ?
				goto	Ain_6stk				; ei, arvestab kohe
				btfss	a6timeron				; jah, kas juba teame ?
				goto	Ain6high1				; ei, k�ivita deb. timer kui vaja
				decfsz	ain6tmr					; debouncetud?
				goto	Ain_7					; ei veel, v�ta j�rgmine sisend
Ain_6stk:		bsf		ainpress6				; jah, v�tame arvesse
;				btfss	in_sticky				; sticky bitti arvestame ?
;				goto	Ain_6setlow				; eip
;				btfsc	ainstuck6				; kas muutus veel teavitamata?
;				goto	Ain6_lstckl				; teavitus saatmata, bitti ei muuda
				bsf		ainstuck6				; blokeerime biti igal juhul
Ain_6setlow:;	bsf		pank1
				bsf		Register1+.1,.6
;				bcf		pank1
Ain6_lstckl:	bcf		a6timeron
				goto	Ain_7					; v�ta j�rgmine sisend
Ain6high:		btfss	ainpress6				; oli k�rge enne ?
				goto	Ain_7					; ei, v�ta j�rgmine sisend
				btfss	in_debounce				; kas debounce't teeb ?
				goto	Ain_6stkh				; ei, arvestab kohe
				btfss	a6timeron				; lahti laskmise debounce k�ib?
				goto	Ain6strt				; ei, paneme k�ima
				decfsz	ain6tmr
				goto	Ain_7					; v�ta j�rgmine sisend
				bcf		ainpress6				; loeme lahti lastuks
Ain_6stkh:	;	btfss	in_sticky				; sticky bitti arvestame ?
;				goto	Ain_6sethi				; eip
;				btfsc	ainstuck6				; kas muutus veel teavitamata?
;				goto	Ain6_lstckh				; teavitus saatmata, bitti ei muuda
				bsf		ainstuck6				; blokeerime biti igal juhul
Ain_6sethi:	;	bsf		pank1
				bcf		Register1+.1,.6
;				bcf		pank1
Ain6_lstckh:	bcf		a6timeron
				goto	Ain_7					; v�ta j�rgmine sisend
Ain6high1:		btfsc	ainpress6
				goto	Ain_7					; v�ta j�rgmine sisend
Ain6strt:		movlw	debtime					; k�ivitame debounce taimeri
				movwf	ain6tmr
				bsf		a6timeron
;				goto	Ain_7					; v�ta j�rgmine sisend
;**** sisendite debounce ***** - > sisend 7 (AIN plokist)
Ain_7:			btfsc	Register275+.1,.7		; kas on sisend ?
				goto	Ain_7sethi				; eip, kirjutab nulli registrisse
				btfss	Register271+.1,.7		; kas on digisisend ?
				goto	Ain_7sethi				; eip, kirjutab nulli registrisse
				btfss	in_sticky
				goto	Ain_7a
				btfsc	ainstuck7
				goto	Ain_8
Ain_7a:			btfsc	Ana7					; sisend 7 madal ?
				goto	Ain7high				; ei, k�rge, seedi seda
				btfss	in_debounce				; kas debounce't teeb ?
				goto	Ain_7stk				; ei, arvestab kohe
				btfss	a7timeron				; jah, kas juba teame ?
				goto	Ain7high1				; ei, k�ivita deb. timer kui vaja
				decfsz	ain7tmr					; debouncetud?
				goto	Ain_8					; ei veel, v�ta j�rgmine sisend
Ain_7stk:		bsf		ainpress7				; jah, v�tame arvesse
;				btfss	in_sticky				; sticky bitti arvestame ?
;				goto	Ain_7setlow				; eip
;				btfsc	ainstuck7				; kas muutus veel teavitamata?
;				goto	Ain7_lstckl				; teavitus saatmata, bitti ei muuda
				bsf		ainstuck7				; blokeerime biti igal juhul
Ain_7setlow:;	bsf		pank1
				bsf		Register1+.1,.7
;				bcf		pank1
Ain7_lstckl:	bcf		a7timeron
				goto	Ain_8					; v�ta j�rgmine sisend
Ain7high:		btfss	ainpress7				; oli k�rge enne ?
				goto	Ain_8					; ei, v�ta j�rgmine sisend
				btfss	in_debounce				; kas debounce't teeb ?
				goto	Ain_7stkh				; ei, arvestab kohe
				btfss	a7timeron				; lahti laskmise debounce k�ib?
				goto	Ain7strt				; ei, paneme k�ima
				decfsz	ain7tmr
				goto	Ain_8					; v�ta j�rgmine sisend
				bcf		ainpress7				; loeme lahti lastuks
Ain_7stkh:;		btfss	in_sticky				; sticky bitti arvestame ?
;				goto	Ain_7sethi				; eip
;				btfsc	ainstuck7				; kas muutus veel teavitamata?
;				goto	Ain7_lstckh				; teavitus saatmata, bitti ei muuda
				bsf		ainstuck7				; blokeerime biti igal juhul
Ain_7sethi:;		bsf		pank1
				bcf		Register1+.1,.7
;				bcf		pank1
Ain7_lstckh:	bcf		a7timeron
				goto	Ain_8					; v�ta j�rgmine sisend
Ain7high1:		btfsc	ainpress7
				goto	Ain_8					; v�ta j�rgmine sisend
Ain7strt:		movlw	debtime					; k�ivitame debounce taimeri
				movwf	ain7tmr
				bsf		a7timeron
Ain_8:	;		movf	Register271+.0,W		; lisa: XORime tulemuse sellega l�bi
T1int_end:
				goto	Int_2
;===============================================================================
; ********************** funktsioonid ******************************************
;===============================================================================
;===============================================================================
; ********************** p�hiluup **********************************************
;===============================================================================
main:			call	init					; prose setup
				call	init_analoog			; analoogpingete m��tmise init
;--- lubame intsid -------------
				banksel	PIE1
				movlw	B'00100001'		 		; luba vaid ser. receive ja T1 intsid
				movwf	PIE1
				banksel	.0
				movf	PORTB,W				
				movlw	B'01101000'				; Perifeeria intsid lubada, samuti TMR0IE ja RBIE int
				movwf	INTCON
				movlw	B'00010000'				; katskestusi ei ole esinenud, RS232 porti ei puudu: read-only
				movwf	PIR1	
				banksel	IOCB
				movlw	0xFF
				movwf	IOCB					; lubame k�ik pordi B muutuse katckestused
				banksel	.0
				clrwdt

				movlw	0x04
				movwf	IPR3					; CCP2 k�rge prioriteediga
				clrf	IPR1					; k�ik muud intsid on madala prioriteediga
				clrf	IPR2
				clrf	IPR4
				clrf	IPR5
				clrf	INTCON2
				bsf		INTCON2,.7				; B-pullupid maha
				bsf		RCON,IPEN				; lubame katkestuste prioriteedid
				bcf		INTCON,TMR0IF			; debuggeri jaox ainult !
				bsf		INTCON,PEIE			

				bsf		INTCON,GIE				; lubame kindral-katckestused
tsykkel:		bcf		PIE1,ADIE
				bcf		PIR1,TXIF				; katkestuse n�ue maha
				btfss	cmd_ok					; oli valiidne k�sk ?
				goto	adc000					; eip, lase edasi
;----------
				btfss	temper					; aeg temperatuuri m��ta ?
				goto	aaa
;	bcf	INTCON,GIE
;	bcf		INTCON,PEIE
				call	GetT					; jepp
;	bsf	INTCON,GIE
;	bsf		INTCON,PEIE
			bcf		temper					; m��tmine tehtud
			movlw	_10sekundiaeg
			movwf	_10sekunditmr
;---------
aaa:				call	command					; jah - t��tle ja t�ida 
;	bcf	Dout0
				call	reset_ser				; tehtud !
				bcf		cmd_ok					; tehtud !
				bcf		PIR1,RC1IF
				banksel	PIE1
				bsf		PIE1,RC1IE				; lubame uue k�su vastuv�ttu
				banksel	.0
adc000:	
				banksel	PIE1
				bsf		PIE1,RC1IE				; lubame uue k�su vastuv�ttu
				banksel	.0

				LFSR	.0,Register2			; ADC muundamine - k�ib k�ik kanalid l�bi
				clrf	meascount
adcchan0:		btfsc	Register275+.1,.0		; kui see bitt on v�ljund, kirjutame tulemuseks 0
				goto	adc0					; ongi
				btfss	Register271+.1,.0		; kui see bitt on digisisend, kirjutame tulemuseks 0
				goto	adc00					; sobib, muundame
adc0:			movlw	0x00
				movff	WREG,Register2
				movff	WREG,Register2+.1
				goto	adcchan1				; v�ta j�rgmine kanal (sisend)
adc00:			clrwdt
				banksel	ADCON0
				movlw	chan1					; on anasisend, muundame
				movwf	ADCON0
				call	measure

adcchan1:		incf	meascount,F				; register 3
				LFSR	.0,Register3
				btfsc	Register275+.1,.1		
				goto	adc1					
				btfss	Register271+.1,.1		
				goto	adc11
adc1:			movlw	0x00
				movff	WREG,Register3
				movff	WREG,Register3+.1
				goto	adcchan2
adc11:			clrwdt
				banksel	ADCON0
				movlw	chan2
				movwf	ADCON0
				call	measure

adcchan2:		incf	meascount,F
				LFSR	.0,Register4			; register 4
				btfsc	Register275+.1,.2		
				goto	adc2					
				btfss	Register271+.1,.2		
				goto	adc22
adc2:			movlw	0x00
				movff	WREG,Register4
				movff	WREG,Register4+.1
				goto	adcchan3
adc22:			clrwdt
				banksel	ADCON0
				movlw	chan3
				movwf	ADCON0
				call	measure

adcchan3:		incf	meascount,F
				LFSR	.0,Register5			; register 5
				btfsc	Register275+.1,.3		
				goto	adc3					
				btfss	Register271+.1,.3		
				goto	adc33
adc3:			movlw	0x00
				movff	WREG,Register5
				movff	WREG,Register5+.1
				goto	adcchan4
adc33:			clrwdt
				banksel	ADCON0
				movlw	chan4
				movwf	ADCON0
				call	measure

adcchan4:		incf	meascount,F		
				LFSR	.0,Register6			; register 6
				btfsc	Register275+.1,.4		
				goto	adc4					
				btfss	Register271+.1,.4		
				goto	adc44
adc4:			movlw	0x00
				movff	WREG,Register6
				movff	WREG,Register6+.1
				goto	adcchan5
adc44:			clrwdt
				banksel	ADCON0
				movlw	chan5
				movwf	ADCON0
				call	measure

adcchan5:		incf	meascount,F
				LFSR	.0,Register7			; register 7
				btfsc	Register275+.1,.5		
				goto	adc5					
				btfss	Register271+.1,.5		
				goto	adc55
adc5:			movlw	0x00
				movff	WREG,Register7
				movff	WREG,Register7+.1
				goto	adcchan6
adc55:			clrwdt
				banksel	ADCON0
				movlw	chan6
				movwf	ADCON0
				call	measure

adcchan6:		incf	meascount,F
				LFSR	.0,Register8			; REGISTER 8
				btfsc	Register275+.1,.6		
				goto	adc6					
				btfss	Register271+.1,.6		
				goto	adc66
adc6:			movlw	0x00
				movff	WREG,Register8
				movff	WREG,Register8+.1
				goto	adcchan7
adc66:			clrwdt
				banksel	ADCON0
				movlw	chan7
				movwf	ADCON0
				call	measure

adcchan7:		incf	meascount,F
				LFSR	.0,Register9			; register 9
				btfsc	Register275+.1,.7		
				goto	adc7					
				btfss	Register271+.1,.7		
				goto	adc77
adc7:			movlw	0x00
				movff	WREG,Register9
				movff	WREG,Register9+.1
				goto	adcchan8
adc77:			clrwdt
				banksel	ADCON0
				movlw	chan8
				movwf	ADCON0
				call	measure
adcchan8:		goto	tsykkel

;===============================================================================
incrpointer:	movlw	.7						; viitab j�rgmise kanali tulemustele
				addwf	FSR1L,F
				btfsc	CARRY
				incf	FSR1H,F
				return
;===============================================================================
; ************** analoogpingete m��tmine ***************************************
;===============================================================================
;					ch1meascnt
;					ch1sum:.2 ; LSB, MSB
;					ch1min:.2 ; MSB, LSB
;					ch1max:.2 ; MSB, LSB
measure:
		banksel	.0
				LFSR	.1,ch1sum				; viitame vahetulemuste registritele
				movf	meascount,W
				mullw	.7
				movf	PRODL,W
				bcf		CARRY
				addwf	FSR1L,F
				btfsc	CARRY
				incf	FSR1H,F
				movf	INDF1,W					; 10 korda m��detud ?
				addlw	.0
				btfsc	ZERO
				goto	keskmista				; jah, arvuta tulemus
sdly0:			movlw	sampletime				; 277us
				movwf	sampledly
sdly:			decfsz	sampledly
				goto	sdly
				banksel	ADCON0
				bsf		ADCON0,GO
meas1:			BTFSC 	ADCON0,GO 
				goto	meas1
				BANKSEL ADRESH 
				movlw	0x0F					; kui >4095 siis n�itu ei salvesta ( on jama !)
				cpfsgt	ADRESH
				goto	meas2					; on OK	
				banksel .0
				movf	POSTINC1,W				; on liiga suur number, suurendame vaid tulemuse m�luviita
				movf	POSTINC1,W
;				call	incrpointer				; vahetulemuste viita suurenda kah
				return
meas2:			decf	POSTINC1,F				; tulemus OK, v�henda keskmistamise kordade loendit
				banksel	ADRESL					; ja liida tulemus summa registrisse
				movf	ADRESL,W
				banksel	.0
				addwf	POSTINC1,F				; NB! LSB ja siis MSB !
				btfsc	CARRY
				incf	INDF1,F
				banksel	ADRESH
				movf	ADRESH,W
				banksel	.0
				addwf	POSTINC1,F
;				movff	POSTINC1,DestH			; loe senine min v��rtus
;				movff	POSTINC1,DestL
				call	comparemin				; v�rdleme min. v��rtusega: kui A < Min,  vaiksem = 1
				btfss	vaiksem
				goto	meas3					; ei ole <, senine min j��b kehtima
				movf	POSTDEC1,W				; tulemus < Min, seivime ta uueks min'iks	
				movf	POSTDEC1,W				; selleks viita tagasi min v��rtusele
				banksel	ADRESH
				movf	ADRESH,W
				banksel	.0
				movwf	POSTINC1
				banksel	ADRESL
				movf	ADRESL,W
				banksel	.0
				movwf	POSTINC1
meas3:;			movff	POSTINC1,DestH			; loe senine max v��rtus
	;			movff	POSTINC1,DestL
				call	comparemax				; v�rdleme max. v��rtusega
				btfss	suurem					; kui A < Max,  Cy = 1
				goto	meas4					; on <, senine max j��b kehtima
				movf	POSTDEC1,W				; tulemus > Max, seivime ta uueks max'iks	
				movf	POSTDEC1,W				; selleks viita tagasi max v��rtusele
				banksel	ADRESH
				movf	ADRESH,W
				banksel	.0
				movwf	POSTINC1
				banksel	ADRESL
				movf	ADRESL,W
				banksel	.0
				movwf	POSTINC1
meas4:			return

keskmista:		movf	POSTINC1,W				; viita summale
				movff	FSR1L,FSR2L				; seivi source viit
				movff	FSR1H,FSR2H
				call	sub1616min				; lahuta min
				movff	FSR2L,FSR1L				; taasta pointer
				movff	FSR2H,FSR1H
				movff	DestL,POSTINC1			; ja kirjuta tulemus tagasi
				movff	DestH,POSTINC1
				movff	FSR2L,FSR1L				; taasta uuesti pointer
				movff	FSR2H,FSR1H
				call	sub1616max				; lahuta ka maksimumv��rtus
				movff	FSR2L,FSR1L				; taasta pointer
				movff	FSR2H,FSR1H
				movff	DestH,POSTINC1			; ja kirjuta tulemus tagasi
				movff	DestL,POSTINC1			; NB! Seekord MSB, LSB !
				movlw	.3						; keskmista ehk jaga 8-ga
				movwf	rombit_idx
keskmista1:		movff	FSR2L,FSR1L				; poindi summale
				movff	FSR2H,FSR1H
				bcf		CARRY
				rrcf	POSTINC1,F
				rrcf	POSTINC1,F
				decfsz	rombit_idx
				goto	keskmista1
				movff	FSR2L,FSR1L				; poindi tulemusele
				movff	FSR2H,FSR1H
				movff	POSTINC1,POSTINC0		; t�sta valmis tulemus oma registrisse
				movff	POSTINC1,POSTINC0		
				movff	FSR2L,FSR1L				; nullime m��tmists�kli v��rtused
				movff	FSR2H,FSR1H
				movf	POSTDEC1,W
				movlw	m��tmisi
				movwf	POSTINC1				; selle kanali keskmistamiste arv
				clrf	POSTINC1				; tulemus
				clrf	POSTINC1
				movlw	MinInitialH
				movwf	POSTINC1				; Min
				movlw	MinInitialL
				movwf	POSTINC1
				movlw	MaxInitialH
				movwf	POSTINC1				; Max
				movlw	MaxInitialL
				movwf	POSTINC1
				return	
; ***********************************************************************
comparemax:		nop
				banksel	ADRESH
				movf   	ADRESH,W		        ; pinge v�rdlemine Max-ga
				banksel	.0
  				subwf  	POSTINC1,W					
  				movwf   rombit_idx				; ajutine seiv
				banksel	ADRESL
  				movf   	ADRESL,W	            
				banksel	.0
  				subwf  	POSTINC1,W	
				goto	compare				
comparemin:		nop
				banksel	ADRESH
				movf   	ADRESH,W		        ; pinge v�rdlemine Min-ga
				banksel	.0
  				subwf  	POSTINC1,W					
  				movwf   rombit_idx				; ajutine seiv
				banksel	ADRESL
  				movf   	ADRESL,W	            
				banksel	.0
  				subwf  	POSTINC1,W					
compare:		btfss  	CARRY		
   				decf  	rombit_idx
  				iorwf  	rombit_idx,W   	        ; Check for Equal to Zero
  				btfsc   ZERO		            ; If Not Zero, Jump Over
   				goto  	compare_eq     		   	; v�rdsed
  				btfsc  	rombit_idx,.7           ; If Number is Negative, execute 
   				goto  	compare_gt	         	; pinge suurem kui etteantud
				bsf		vaiksem					; pinge madalam kui etteantud
				bcf		suurem
				bcf		vordne
				return
compare_gt:		bcf		vaiksem
				bsf		suurem
				bcf		vordne
				return
compare_eq:		bcf		vaiksem
				bcf		suurem
				bsf		vordne
				return
;*************************************************************************
;       SourceH:SourceL - Number to be subtracted
;       Carry - NOT( Borrow to be subtracted )
;       DestH:DestL - Number to be subtracted FROM
;Out    DestH:DestL - Result
;       Carry - NOT( Borrow result)
; Dest=Dest-Source
;*************************************************************************
sub1616min:		movff	POSTINC1,DestL			; summa registri sisust lahutame
				movff	POSTINC1,DestH			; NB! Summa on LSB, MSB j�rjekorras
				movff	POSTINC1,SourceH		; min registri sisu
				movff	POSTINC1,SourceL
				goto	sub1616
sub1616max:		movff	POSTINC1,DestL			; summa registri sisust lahutame
				movff	POSTINC1,DestH			; NB! Summa on LSB, MSB j�rjekorras
				movf	POSTINC1,W
				movf	POSTINC1,W
				movff	POSTINC1,SourceH		; max registri sisu
				movff	POSTINC1,SourceL
sub1616:        movff   SourceL,WREG
        		subwf   DestL
        		movff   SourceH,WREG
        		btfss   CARRY
        		incfsz  SourceH,W
        		subwf   DestH           		; dest = dest - source, WITH VALID CARRY (although the Z flag is not valid).                                
				return
;*************************************************************************
; MB_CRC16 -	Will calculate 16bit CRC for MODBUS RTU Packets
;
;		Input: 	Byte for CRC in W
;		Output:	Original byte in W, 
;			CRC16_HI and CRC16_LO new value of CRC16
;*************************************************************************
mb_crc16:		movwf	mb_del1
				movwf	mb_temp2				; store W
				movlw	.8						; 8 bits
				movwf	mb_temp1
				movf	mb_temp2,W				; fetch W
Crc_Get_Bit:	rrcf		mb_temp2,F				; bit in C
				movf	mb_temp2,W				; value to W
				bnc	crc1;skpnc
				goto	Crc_In_1
crc1:			btfss	_RS485chk,.0			; lowest bit set ?
				goto	Crc_Cont				; goto count with C=0
				bsf		CARRY
				goto	Crc_Cont				; goto count with C=1
Crc_In_1:		btfsc	_RS485chk,.0			; lowest bit zero ?
				bcf		CARRY					; if no, C=0 = complement
Crc_Cont:		bc		crc2;skpc
				goto	Crc_Shift				; if C=0 only shift
crc2:			btfsc	_RS485chkH,.6			; complement 15th bit of CRC
				goto	Crc1
				bsf		_RS485chkH,.6			; if clear, set
				goto	Crc2
Crc1:			bcf		_RS485chkH,.6			; if set, clear
Crc2:			btfsc	_RS485chk,.1			; complement 2nd bit of CRC
				goto	Crc3
				bsf		_RS485chk,.1
				goto	Crc_Shift
Crc3:			bcf		_RS485chk,.1
Crc_Shift:		rrcf		_RS485chkH,F			; 16bit rotate
				rrcf		_RS485chk,F
				movf	mb_temp2,W
				decfsz	mb_temp1,F
				goto	Crc_Get_Bit
				movf	mb_del1,W				; fetch the original byte
				return
;===============================================================================
; ************************* saadame liinile vastuse ****************************
;===============================================================================
SendCHAR:		movwf	Char;sendtemp				; seivi saadetav
				movwf	countH
;				bsf		pank1
				btfsc	Register273+.1,.7		; bit7 paarsus. 0=EVEN,1=NO PARITY
				goto	snd_exit
;				bcf		pank1
; kalkuleeri paarsuse bitt
				movlw	.8
				movwf	countL
				clrf	adrtemp					; 1-tede loendi
				bcf		CARRY
parity:			rrcf		countH,F
				btfsc	CARRY
				incf	adrtemp,F
				decfsz	countL
				goto	parity
				bcf		CARRY
				btfsc	adrtemp,.0
				bsf		CARRY
;				banksel	TXSTA1					; paarsuse bitt on n��d TXSTA,TX9D-s
;			bsf		pank
				banksel	TXSTA1
				bcf		TXSTA1,TX9D
				btfsc	CARRY
				bsf		TXSTA1,TX9D
				banksel	.0
snd_exit:
;		banksel	.0
snd_exita:		btfss   PIR1,TXIF     			; saatja valmis ?   
			    goto    snd_exita
				movf	Char,W;sendtemp,W				; saadetav bait meelde tuletada
				banksel	TXREG1
				movwf   TXREG1    				; saada!
				banksel	TXSTA1
snd_exit1:		btfss	TXSTA1,TRMT				; kas saatja nihkeregister t�hi (bait prosest v�ljas)?
				goto	snd_exit1
				banksel	.0
;				bsf		pank
				movf	Char,W;sendtemp,W				; taasta saadetav
;				bcf		pank
			    return
;===============================================================================
;				org		0x800
setup_serial:;	nop
;				bsf		pank1
				movff	Register273+.1,WREG	
				andlw	0x07
				movff	WREG,Register273
;				bcf		pank1
				movlw	.71						; baudrate = (9600 @ 11.0592 MHz)
				banksel	SPBRG1	
				movwf	SPBRG1		
				banksel	.0
;				bsf		pank1
				movff	Register273,WREG
;				bcf		pank1
				sublw	.1						; kas on 9600 ?
				btfsc	ZERO
				goto	initser_par				; jah
				movlw	.35						; baudrate = (19200 @ 11.0592 MHz)
				banksel	SPBRG1	
				movwf	SPBRG1		
				banksel	.0
;				bsf		pank1
				movff	Register273,WREG
;				bcf		pank1
				sublw	.2						; kas on 19200 ?
				btfsc	ZERO
				goto	initser_par				; jah
				movlw	.17						; baudrate = (38400 @ 11.0592 MHz)
				banksel	SPBRG1	
				movwf	SPBRG1		
				banksel	.0
;				bsf		pank1
				movff	Register273,WREG
;				bcf		pank1
				sublw	.3						; kas on 38400 ?
				btfsc	ZERO
				goto	initser_par				; jah
				movlw	.11						; baudrate = (57600 @ 11.0592 MHz)
				banksel	SPBRG1	
				movwf	SPBRG1		
				banksel	.0
;				bsf		pank1
				movff	Register273,WREG
;				bcf		pank1
				sublw	.4						; kas on 57600 ?
				btfsc	ZERO
				goto	initser_par				; jah
				movlw	.5						; baudrate = (115200 @ 11.0592 MHz)
				banksel	SPBRG1	
				movwf	SPBRG1		
				banksel	.0
;				bsf		pank1
				movff	Register273,WREG
;				bcf		pank1
				sublw	.5						; kas on 115200 ?
				btfsc	ZERO
				goto	initser_par				; jah
setup_serial0:
				movlw	.35						; mingi kamm, v�tab baudrate = (19200 @ 11.0592 MHz)
				banksel	SPBRG1	
				movwf	SPBRG1		
				banksel	.0
;				bsf		pank1
				movff	Register273+.1,WREG		; parandame vea: v�tab vaikimisi seriali seaded aga ei muuda debounce ja sticky bitte ! Samuti ei n�pi Wiegandi bitte !!!
;				bcf		pank1
				andlw	0x78
;				bsf		pank1
				movff	WREG,Register273
;				bcf		pank1
				movlw	defaultserial
				andlw	0x07
;				bsf		pank1
			addwf	Register273,W
				movff	WREG,Register273+.1			
;				bcf		pank1
initser_par:;	bsf		pank1
				movlw	0x00
				movff	WREG,Register273		; selle solkisime ��, n��d nulli
				movlw	B'01100111'				; paarsuse kalkuleerimine - eeldame: 9 bitine saade
				btfsc	Register273+.1,.7		; paarsus even (0) v�i puudub (1) ?
				movlw	B'00100110'				; paarsus puudub: 8 bitine saade
;				bcf		pank1
				banksel	TXSTA1
				movwf	TXSTA1
				banksel	.0
				movlw	B'11010000'				; sama RCSTA jaoks: eeldame paarsust st. 9-bitist saadet
;				bsf		pank1
				btfsc	Register273+.1,.7		
				movlw	B'10010000'			
;				bcf		pank1
				banksel	RCSTA1
				movwf	RCSTA1
				banksel	.0
				return
;===============================================================================
;******************* EEPROMi funksioonid ***************************************
;===============================================================================
;===============================================================================
; **** Loeb parameetrid  EEPROMist ja kirjutab s�steemi ************************
;===============================================================================
Read_Setup:		movlw	LOW(e_ADR)				; loe oma modbussi aadress
				banksel	EEADR
				movwf	EEADR
				clrf	EEADRH
				call	Read_EEPROM					
				movff	WREG,Register274+.1
				movlw	0x00
				movff	WREG,Register274
				call	Read_EEPROM				; loe oma ID (H)	
				movff	WREG,Register258+.1
				movlw	0x00
				movff	WREG,Register258
				call	Read_EEPROM				; loe oma ID (L)	
				movff	WREG,Register259+.1
				movlw	0x00
				movff	WREG,Register259
				call	Read_EEPROM				; Analoog-Pull-up'ide seis ehk anav�ljundite seis
				movff	WREG,Register272+.1		
				movff	WREG,Register0+.1		; ja v�ljundite seisu n�itavasse registrisse (NB! ainult stardil)
				call	Read_EEPROM					
				movff	WREG,Register272+.0		; Digital-Pull-up'ide seis
				movff	WREG,Register0			; ja v�ljundite seisu n�itavasse registrisse
				movwf	DoutPort				; ja kohe v�ljunditesse kah (NB! ainult stardil)
				call	Read_EEPROM				; seriali parameetrid	
				movff	WREG,Register273+.1
				clrf	Register273	
				call	Read_EEPROM				; ANA-pordi (UIO) suund, 1= v�ljund, bitthaaval)	
				movff	WREG,Register275+.1		
				call	Read_EEPROM				; analoogpordi seisund stardil - analoog v�i digi. 1= digi
				movff	WREG,Register271+.1
				movlw	0x00
				movff	WREG,Register271
				call	Read_EEPROM				; seadme t��p	
				movff	WREG,Register256+.1
				movlw	0x00
				movff	WREG,Register256
				call	Read_EEPROM				; firmware nr HIGH
				movff	WREG,Register257+.0
				call	Read_EEPROM				; firmware nr LOW
				movff	WREG,Register257+.1

; *** resetid ****
				call	Read_EEPROM				; reset 1 ajad: viiteaeg peale pingestamist				
				movff	WREG,Register276
				movwf	reset1strttmrH
				call	Read_EEPROM					
				movff	WREG,Register276+.1
				movwf	reset1strttmrL
				call	Read_EEPROM				; reseti pulsi kestus				
				movff	WREG,Register277
				movwf	reset1pulsetmr
				call	Read_EEPROM					
				movff	WREG,Register277+.1		; side kadumise viiteaeg
				movwf	reset1dlytmr
				call	Read_EEPROM				; reset 2 ajad: viiteaeg peale pingestamist				
				movff	WREG,Register278
				movwf	reset2strttmrH
				call	Read_EEPROM					
				movff	WREG,Register278+.1
				movwf	reset2strttmrL
				call	Read_EEPROM				; reseti pulsi kestus				
				movff	WREG,Register279
				movwf	reset2pulsetmr
				call	Read_EEPROM					
				movff	WREG,Register279+.1		; side kadumise viiteaeg
				movwf	reset2dlytmr
; *** XORimine ****
				call	Read_EEPROM				; juhitav aktiivne nivoo ANA-pordi sisendis
				movff	WREG,Register271+.0
; *** ADC ****
				call	Read_EEPROM				; ADC registri ADCON1 bitid
				andlw	0x30					; lubame vaid Vref-i puudutavaid bitte
				movff	WREG,Register270+.1
				BANKSEL ADCON1 
				MOVWF 	ADCON1 					; konfime kohe ADC ka �ra
				banksel	.0
				movlw	0x00
				movff	WREG,Register270
; *** PWM ****
				call	Read_EEPROM				; PWMi perioodi register
				movff	WREG,Register150+.0
				call	Read_EEPROM			
				movff	WREG,Register150+.1
				call	Read_EEPROM				; PWMi konfi register
				movff	WREG,Register151+.0
				call	Read_EEPROM			
				movff	WREG,Register151+.1
				call	reset_pwm				; initsialiseerime PWMi
;			call	reset_chanls			; taasta pulsside kestused
				goto	setup_port				; kombineeri DO ja UIO juhtimine ja vastavate registrite sisud
				return
reset_pwm:		movff	Register150+.0,pwmtmp+.1; pertick'is on periood*4 sammu
				movff	Register150+.1,pwmtmp+.2
				clrf	pwmtmp+.0
				bcf		CARRY
				rlcf	pwmtmp+.2,F				; korrutame 4-ga
				rlcf	pwmtmp+.1,F
				rlcf	pwmtmp+.0,F
				rlcf	pwmtmp+.2,F
				rlcf	pwmtmp+.1,F
				rlcf	pwmtmp+.0,F
				movf	pwmtmp+.0,W				; Periood = 0 ?
				addlw	.0
				btfss	ZERO
				goto	reset_pwm1				; eip
				movf	pwmtmp+.1,W				
				addlw	.0
				btfss	ZERO
				goto	reset_pwm1		
				movf	pwmtmp+.2,W				
				addlw	.0
				btfss	ZERO
				goto	reset_pwm1
;---CHG
		bsf		T0CON,TMR0ON
		goto	rp1
;;;;;				bcf		T0CON,TMR0ON			; jah, PWM seisma
;;;;;				goto	reset_pwm2
reset_pwm1:		bsf		T0CON,TMR0ON			; PWMi taimer k�ima
reset_pwm2:		movff	pwmtmp+.0,masterpertick+.0
				movff	pwmtmp+.1,masterpertick+.1
				movff	pwmtmp+.2,masterpertick+.2
				movff	pwmtmp+.0,periodtick+.0
				movff	pwmtmp+.1,periodtick+.1
				movff	pwmtmp+.2,periodtick+.2
reset_phase:	movff	Register150+.0,mphasetick+.0; faasis on "periood" sammu
				movff	Register150+.1,mphasetick+.1
				movff	mphasetick+.0,phasetick+.0
				movff	mphasetick+.1,phasetick+.1
				movlw	0x00	
				movff	WREG,phasecounter
				movlw	.4						; jaap, taasta 1ms loendi
				movwf	_1mscount
rp1:			clrf	Register149out+.0		; aeg perioodi algusest
			clrf	Register149out+.1
			return

reset_chanls:	LFSR	.0,pwm0set				; taasta pulsside kestused
				LFSR	.1,pwm0work
				movlw	.16	
				movff	WREG,_1mscount			; ajutine loendi
reset_chanls1:	btfsc	INDF0,.7				; �hekordse pulsi kestust ei taasta, olgu 0
				goto	reset_chanls2
				movf	POSTINC0,W
				movf	POSTINC1,W
				movf	POSTINC0,W
				movf	POSTINC1,W
				goto	reset_chanls3
reset_chanls2:	movff	POSTINC0,POSTINC1
				movff	POSTINC0,POSTINC1
reset_chanls3:	decfsz	_1mscount
				goto	reset_chanls1
				movlw	.4					; jaap, taasta 1ms loendi
				movwf	_1mscount
;			clrf	Register149out+.0		; aeg perioodi algusest
;			clrf	Register149out+.1
				return
reset_per:		movlw	0x00
				movff	WREG,periodtick+.0		; nulli perioodi loendi et periood saax kohe alata
				movff	WREG,periodtick+.1
				movff	WREG,periodtick+.2
				movlw	.0					
				movwf	TMR0L
;				clrf	RegX+.0					; nulli PWMi loogika register
;				clrf	RegX+.1
				LFSR	.0,Register151+.1
				movlw	0x00
				movwf	INDF0
;				btfsc	INDF0,.0				; nulli PWMi v�ljundid
;				bsf		PORTA,.0
;				btfsc	INDF0,.1				
;				bsf		PORTA,.1
;				btfsc	INDF0,.2				
;				bsf		PORTA,.2
;				btfsc	INDF0,.3				
;				bsf		PORTA,.3
;				btfsc	INDF0,.4				
;				bsf		PORTA,.5
;				btfsc	INDF0,.5				
;				bsf		PORTE,.0
;				btfsc	INDF0,.6				
;				bsf		PORTE,.1
;				btfsc	INDF0,.7				
;				bsf		PORTE,.2

;				btfsc	INDF0,.0				; nulli Registri 0 vastavad bitid
;				bcf		Register0+.1,.0			
;				btfsc	INDF0,.1			
;				bcf		Register0+.1,.1			
;				btfsc	INDF0,.2			
;				bcf		Register0+.1,.2			
;				btfsc	INDF0,.3			
;				bcf		Register0+.1,.3			
;				btfsc	INDF0,.4			
;				bcf		Register0+.1,.4			
;				btfsc	INDF0,.5			
;				bcf		Register0+.1,.5			
;				btfsc	INDF0,.6			
;				bcf		Register0+.1,.6			
;				btfsc	INDF0,.7			
;				bcf		Register0+.1,.7			

				movf	POSTDEC0,W				; j�rgmised 8 kanalit
				movlw	0x00
				movwf	INDF0
;				btfsc	INDF0,.0				
;				bsf		PORTD,.0
;				btfsc	INDF0,.1				
;				bsf		PORTD,.1
;				btfsc	INDF0,.2				
;				bsf		PORTD,.2
;				btfsc	INDF0,.3				
;				bsf		PORTD,.3
;				btfsc	INDF0,.4				
;				bsf		PORTD,.4
;				btfsc	INDF0,.5				
;				bsf		PORTD,.5
;				btfsc	INDF0,.6				
;				bsf		PORTD,.6
;				btfsc	INDF0,.7				
;				bsf		PORTD,.7
;
;				btfsc	INDF0,.0			
;				bcf		Register0+.0,.0			
;				btfsc	INDF0,.1			
;				bcf		Register0+.0,.1			
;				btfsc	INDF0,.2			
;				bcf		Register0+.0,.2			
;				btfsc	INDF0,.3			
;				bcf		Register0+.0,.3			
;				btfsc	INDF0,.4			
;				bcf		Register0+.0,.4			
;				btfsc	INDF0,.5			
;				bcf		Register0+.0,.5			
;				btfsc	INDF0,.6			
;				bcf		Register0+.0,.6			
;				btfsc	INDF0,.7			
;				bcf		Register0+.0,.7			
				return
;===============================================================================
; **** Salvestab parameetrid EEPROMisse ja kirjutab s�steemi *******************
;===============================================================================
Save_Setup:		movlw	LOW(e_ADR)				; loe oma aadress
				banksel	EEADR
				movwf	EEADR
				clrf	EEADRH
				movff	Register274+.1,WREG		; aadress
				call	Wr_EEPROM				; seivi
				incf	EEADR,F					; j�rgmine aadress (ID-d ei kirjuta!)
				incf	EEADR,F					; j�rgmine aadress
				movff	Register272+.1,WREG		; PU-d - ANA pordile
				call	Wr_EEPROM	
				movff	Register272+.0,WREG		; PU-d DO - pordile
				call	Wr_EEPROM
				call	setup_serial			; seriali seaded kohe serial porti
				movff	Register273+.1,WREG		; seriali seaded uuesti sest kui oli viga, parandab pordi rutiin selle �ra
				call	Wr_EEPROM				
				movff	Register275+.1,WREG		; IOD ehk ANA-pordi suund
				call	Wr_EEPROM				; kirjutame EEPROMi aga arvestame alles peale ANCONxi s�ttimist !
				movff	Register271+.1,WREG		; pordi Ana/Digi omadused (1=digi)
				call	Wr_EEPROM				; seivib EEPROMi
; *** resetid ****
				movlw	LOW(e_reset1)			; reset 1 ajad
				banksel	EEADR
				movwf	EEADR
				clrf	EEADRH
				movff	Register276,WREG		; viiteaeg peale pingestamist
				movwf	reset1strttmrH
				call	Wr_EEPROM
				movff	Register276+.1,WREG		
				movwf	reset1strttmrL
				call	Wr_EEPROM
				movff	Register277,WREG		; reseti pulsi kestus
				movwf	reset1pulsetmr
				call	Wr_EEPROM
				movff	Register277+.1,WREG		; side kadumise viiteaeg	
				movwf	reset2dlytmr
				call	Wr_EEPROM
				movlw	LOW(e_reset2)			; reset 2 ajad
				movwf	EEADR
				clrf	EEADRH
				movff	Register278,WREG		; viiteaeg peale pingestamist
				movwf	reset2strttmrH
				call	Wr_EEPROM
				movff	Register278+.1,WREG		
				movwf	reset2strttmrL
				call	Wr_EEPROM
				movff	Register279,WREG		; reseti pulsi kestus
				movwf	reset2pulsetmr
				call	Wr_EEPROM
				movff	Register279+.1,WREG		; side kadumise viiteaeg	
				movwf	reset2dlytmr
				call	Wr_EEPROM
; *** XORimine ****
				movff	Register271+.0,WREG		; ANA-pordi v�ljundi aktiivse nivoo juhtimine
				call	Wr_EEPROM			; see oli puudu !!!
; *** ADC ****
				movff	Register270+.1,WREG		; ADC konf
				andlw	0x30	
				call	Wr_EEPROM
				movff	Register270+.1,WREG		; ADC konf uuesti
				andlw	0x30	
				BANKSEL ADCON1 
				MOVWF 	ADCON1 					; konfime kohe ADC ka �ra
				banksel	.0
; *** PWM ***
				movff	Register150+.0,WREG		; PWMi perioodi register
				call	Wr_EEPROM
				movff	Register150+.1,WREG
				call	Wr_EEPROM				
				movff	Register151+.0,WREG
				call	Wr_EEPROM				; PWMi konfi register
				movff	Register151+.1,WREG
				call	Wr_EEPROM				
;===============================================================================
; kombineerib DO ja UIO juhtimise ja vastavate registrite sisud
;===============================================================================
setup_port:		movf	Register275+.1,W		; IOD ehk ANA-pordi suund. 1 = OUT
				andwf	ANCON0,F				; k�ik sisendid tuleb teha digiks sest analoogsisend annab alati 0.
				comf	Register275+.1,W		; prosel on 1 = IN aga kood tahab et olex vastupidi
				movwf	datatemp				; h��lestab ANA-pordi suuna ringi IOD j�rgi: ana-port on pordis A ja E !
				bsf		datatemp,.7				; kvarts
				bsf		datatemp,.6				; kvarts
				bsf		datatemp,.5				; sisend 4
				btfsc	Register275+.1,.4
				bcf		datatemp,.5
				bsf		datatemp,.4				; vaba
				movf	datatemp,W
				banksel	TRISA
				movwf	TRISA					; pordi suund rakendatakse kohe: port A
				banksel	.0
				swapf	Register275+.1,W		; analoogpordi suund, 1= v�ljund, 0= sisend
				movwf	datatemp
				comf	datatemp,W
				movwf	datatemp
				rrcf	datatemp,W
				andlw	0x07
				banksel	TRISE
				movwf	TRISE					; ja porti E
				banksel	.0
; pordi Ana/Digi omadused. 1 = DIGI, 0 = ANA
				comf	Register271+.1,W		; 1=ANA: v�ljund ei saa olla analoog !
				movwf	Registertemp7
				comf	Register275+.1,W		; 0 = OUT
				andwf	Registertemp7,W
				banksel	ANCON0
				movwf	ANCON0					; prose ANCONx registrisse
				clrf	ANCON1
				banksel	.0

				movf	Register271+.1,W		; analoogpinnile vastav bitt registris 1+1 (UIO sisendid) => 0
				banksel	Register1
				andwf	Register1+.1,F
				banksel .0

				movff	Register0+.1,WREG		; taastame UIO v�ljundite seisu vastava registri j�rgi
			andwf	Register271+.1,W		; maskeerime ANA/digi oamdusega (digi=1)
				movwf	Registertemp7
				andlw	0x0F
				movwf	PORTA					
				btfsc	Registertemp7,.4		; kombineerime baidid ANA-pordi juhtimisex	
				bsf		PORTA,.5
				bcf		CARRY
				rrcf	Registertemp7,F
				swapf	Registertemp7,W
				andlw	0x07
				movwf	PORTE
				return
;===============================================================================
;  EEPROMi kirjutamine
;===============================================================================
Wr_EEPROM:		movwf	EEDATA
				BCF 	EECON1,EEPGD
				BSF 	EECON1,WREN
				MOVLW 	55h
				MOVWF 	EECON2
				MOVLW 	0AAh
				MOVWF 	EECON2
				BSF 	EECON1,WR
				BTFSC 	EECON1,WR
				GOTO 	$-2
				BCF 	EECON1,WREN
				incf	EEADR,F					; j�rgmine aadress
				movff	EEDATA,WREG				; meenutame datat
				return
;===============================================================================
;  EEPROMist lugemine
;===============================================================================
Read_EEPROM:	BCF 	EECON1,EEPGD			; Point to DATA memory
				bsf		EECON1,RD			 	; loe!
				movff	EEDATA,WREG
				incf	EEADR,F					; j�rgmine aadress
				return
;===============================================================================
; ******* I-n��bi funktsioonid *******
;===============================================================================
;===============================================================================
; ******* n��bi lugemine nulli *******
;===============================================================================
Search_1Wire_init
              clrf      work0            ;clear the work area to zeros
              clrf      work1
              clrf      work2
              clrf      work3
              clrf      work4
              clrf      work5
              clrf      work6
              clrf      work7
              clrf      last_discrep     ;init the bit position of last descrepancy
              clrf      done_flag        ;set status to not done
 			  clrf	    DS1820found	 ; esialgu pole �htegi Dallase m�lakat leitud ;)
			  clrf		DS2438found
             return
;===============================================================================
; ******* otsi n��pi *******
;===============================================================================
Search_1Wire: clrf      return_value     		; set return flag to false
              btfss     done_flag,0      		; Are we done yet?
              goto      Do_reset         		; No, start a search
              clrf      done_flag        		; Yes, init this for next time???
              goto      Wrapup           		; and get on out of here

Do_reset:     call      Reset_1wire				; n��bile saabast
              clrf      rombit_idx       		; set rom bit index to 1
              incf      rombit_idx, F
              clrf      curr_discrep     		; set descrepancy marker to 0
              movlw     0xF0            		; search rom k�sk
              movwf     OneWireByte
              call      Sendbyte_1wire   		; saada k�sk
Get_2bits             
              call      Readbit_1wire    		; read bit a from bus and
              movwf     bits             		; save it
              bcf		CARRY                  
              rlcf       bits,F          		; shift bit A over to the left
              call      Readbit_1wire    		; read bit B from bus and 
              iorwf     bits,F			        ; save it

;              movlw     HIGH lookup_1x
;              movwf     PCLATH
              movf      bits,W
			  addlw		.0
			  btfsc	     ZERO
              goto      bits_00          		; collision detected
              movf      bits,W
			  sublw		.1
			  btfsc	     ZERO
              goto      bits_01          		; 0 read
              movf      bits,W
			  sublw		.2
			  btfsc	     ZERO
              goto      bits_10          		; 1 read
              goto      bits_11          		; nobody there

;lookup_1x:    addwf     PCL, F           		; decode the bits read
;              goto      bits_00          		; collision detected
;              goto      bits_01          		; 0 read
;              goto      bits_10          		; 1 read
;              goto      bits_11          		; nobody there

bits_00:      movf      rombit_idx,W			; collision detected
              xorwf     last_discrep, W
              btfss     ZERO			       	; Does rombit_idx = last_discrep
              goto     	_chknxt          		; No, check other stuff
              movlw     b'10'            		; Yes, pretend we read a 1
              movwf     bits
              call      Store_Bit        		;Store a 1 into work area
              goto      ack_bus

_chknxt:      movf      last_discrep,W		    ; get current bit position
              subwf     rombit_idx, W    		; compare to last discrepancy position
              btfss     CARRY       			; is rombit_idx > last_discrep
              goto     _chknx2          		; No, 
              movlw     b'01'            		; Yes, pretend we read a 0
              movwf     bits
              call      Store_Bit
              movf      rombit_idx,W			;set discrepancy marker to rombit_idx       
              movwf     curr_discrep
              goto      ack_bus

_chknx2:
;does rombits(rombit_idx) = 0 ?
              call      GetWorkBit       		; get bit located at work(rombit_idx)
              movwf     TEMP0            		; and save

;              movlw     HIGH lookup_2x
;              movwf     PCLATH
              movf      TEMP0,w

lookup_2x:  ;  addwf     PCL, F           		; quik test for 0 or 1
			  addlw		.0
			  btfsc		ZERO
              goto      _zero            		; was 0
_one:                                     		; was 1
              movlw     b'10'            		; Pretend we read a 1
              movwf     bits
              goto      ack_bus
_zero:
              movlw     b'01'            		; Pretend we read a 0
              movwf     bits
              movf      rombit_idx,W			; set discrepancy marker to rombit_idx
              movwf     curr_discrep
              goto      ack_bus
              
bits_01:      goto      _storit					; 0 received
bits_10:                                  		; 1 received
_storit:      call      Store_Bit        		; Save it into work area at rombit_idx offset

; Send rombit(rombit_idx) to 1-wire bus
ack_bus:	  bcf		CARRY
              rrcf       bits,W          		; get bit A into W
              call      Sendbit_1wire    		; send bit A to wire
; Increment rombit_idx
              incf      rombit_idx, F    		; bump pointer to next location
; Is rombit_idx > 64?
              movf      rombit_idx,W	        ; see if index > 64
              sublw     d'65'            		; (i.e. = 65)
              btfss     ZERO
              goto      Get_2bits        		; if not, loop back for more

              movf      curr_discrep,W			; set last discrepancy to descrepancy marker     
              movwf     last_discrep
              btfsc     ZERO   					; is last discrepancy = 0?  
              incf      done_flag, F     		; yes, set done flag
              movlw     d'1'             		; either way, set return value to true
              movwf     return_value
              goto      Wrapup
              
bits_11:      clrf      last_discrep			; nothing answered on the bus
              goto      Wrapup             
Wrapup:       return
;===============================================================================
; ******* I-n��bi reset *******
;===============================================================================
Reset_1wire:	bcf 	_1WDAT               	; madalax
;	bsf	INTCON,GIE
				call	wait					; 375 uS
				call	wait					; kokku ca. 750 uS
	            bsf 	_1WDAT	                ; k�rgex
				call	wait
				call	wait					; kokku ca. 750 uS
;	bcf	INTCON,GIE
              	return
;===============================================================================
; ******* saadab k�subaidi registrist OneWireByte I-n��bile *******
;===============================================================================
Sendbyte_1wireA:movwf	OneWireByte				; seivi saadetav
;	bcf	INTCON,GIE
Sendbyte_1wire:	movlw	0x08					; 8 bitti
              	movwf   TEMP0
_send_1w_lp:    clrf    TEMP1            		; Clear work area
              	bcf     TEMP1,0			        ; Assume sending 0
              	bcf		CARRY                   ; Fill new bits with 0
              	rrcf     OneWireByte,F   		; Load CARRY with bit to send
              	btfsc   CARRY			        ; See what we are sending
               	bsf     TEMP1,0         		; must be a 1
              	movf    TEMP1,W		            ; Load up the bit and
              	call    Sendbit_1wire    		; send it out
              	decfsz  TEMP0,F         		; 8 bitti ok ?
               	goto     _send_1w_lp      		; vara veel
; 	bsf	INTCON,GIE
             	return
;===============================================================================
; ******* saadab k�subiti I-n��bile *******
;===============================================================================
Sendbit_1wire:	bcf	INTCON,GIE
  movwf   TEMP1
              	bcf     _1WDAT	         		; madalax
              	btfsc   TEMP1,0 
               	bsf     _1WDAT	        		; pulsi l�pp kui bitt oli 1
				movlw	.58						; (Tslot + Trec) - Tlow1
				call	wait_x
	            bsf     _1WDAT        			; pulsi l�pp kui bitt oli 0
              	nop    							; Trec
	bsf	INTCON,GIE
	            return
;===============================================================================
; ******* loeb vastusbaidi I-n��bist registrisse OneWireByte *******
;===============================================================================
Readbyte_1wire:	movlw   d'8'
;	bcf	INTCON,GIE
               	movwf   TEMP0
_read_1w_lp:  	call    Readbit_1wire	   
              	movwf   TEMP1            		; Save the bit read
               	bcf		CARRY                   ; Assume bit is 0
              	btfsc   TEMP1,0         		; Check bit
               	bsf		CARRY                  	; its a 1
              	rrcf     OneWireByte, F   		; Rotate bit into IO area
	            decfsz  TEMP0,F		 	        ; 8 bitti OK?
               	goto     _read_1w_lp      		; vara veel
              	movf    OneWireByte,W		    ; jah, tulemus W-sse
; 	bsf	INTCON,GIE
             	return                     ;
;===============================================================================
; ******* loeb vastusbiti I-n��bist *******
;===============================================================================
Readbit_1wire:	bcf	INTCON,GIE
  bcf    	_1WDAT	         		; taktipulss - madalax
				nop
	            bsf    	_1WDAT	         		; taktipulss - k�rgex
;				bsf		pank					; l�litame n��bi otca sisendiks
				bsf		TRIS_1WDAT
;				bcf		pank
	            clrf    TEMP1            		; Assume incomming bit is 0
				movlw	.12						; oota 12uS
				call	wait_x
 	            btfsc   _1WDAT	         		; What are we receiving?
                bsf     TEMP1,0			        ; must be a 1
				movlw	.47						; oota 47uS
				call	wait_x
              	movf    TEMP1,W
;				bsf		pank					; l�litame n��bi otca v�ljundiks
				bcf		TRIS_1WDAT
;				bcf		pank
	bsf	INTCON,GIE
	            return
;===============================================================================
; Store_Bit v�tab bitt A (bits.1) ja salvestab alasse work, offset rombit_idx
; bits.1 -> work0..work7(rombit_idx)
;===============================================================================
Store_Bit:    call      SetupFSR         		; converdi rombit_idx m�lu aadressiks
;              movlw     HIGH SetWorkBit
;              movwf     PCLATH            
              rrcf       bits, W          		; loe bit.1 value right justified into W
              andlw     b'00000001'     		; lolle aadresse ei luba
			  btfsc		ZERO
;              addwf     PCL, F           		; quick test for 0 or 1
              goto      ClrWorkBit       		; must be 0 (clr bit)
SetWorkBit:	                               		; must be 1 (set bit)
			movf	TEMP2,W
			sublw	.0
			btfsc	ZERO
			goto	set0
			movf	TEMP2,W
			sublw	.1
			btfsc	ZERO
			goto	set1
			movf	TEMP2,W
			sublw	.2
			btfsc	ZERO
			goto	set2
			movf	TEMP2,W
			sublw	.3
			btfsc	ZERO
			goto	set3
			movf	TEMP2,W
			sublw	.4
			btfsc	ZERO
			goto	set4
			movf	TEMP2,W
			sublw	.5
			btfsc	ZERO
			goto	set5
			movf	TEMP2,W
			sublw	.6
			btfsc	ZERO
			goto	set6
			goto	set7

;              bcf		CARRY
;              rlcf       TEMP2, W         		; get bit position * 2 (0, 2, 4 .. 14)
;              addwf     PCL, F           		; bump PC, turn bit on then return
set0:              bsf       INDF0, 7
              return
set1:              bsf       INDF0, 6
              return
set2:              bsf       INDF0, 5
              return
set3:              bsf       INDF0, 4
              return
set4:              bsf       INDF0, 3
              return
set5:              bsf       INDF0, 2
              return
set6:              bsf       INDF0, 1
              return
set7:              bsf       INDF0, 0
              return
ClrWorkBit
;              movlw     HIGH ClrWorkBit
;              movwf     PCLATH
;              bcf		CARRY
;              rlcf       TEMP2, W         		; get bit position * 2 (0, 2, 4 .. 14)
;              addwf     PCL, F           		; bump PC and turn it off then return
			movf	TEMP2,W
			sublw	.0
			btfsc	ZERO
			goto	clr0
			movf	TEMP2,W
			sublw	.1
			btfsc	ZERO
			goto	clr1
			movf	TEMP2,W
			sublw	.2
			btfsc	ZERO
			goto	clr2
			movf	TEMP2,W
			sublw	.3
			btfsc	ZERO
			goto	clr3
			movf	TEMP2,W
			sublw	.4
			btfsc	ZERO
			goto	clr4
			movf	TEMP2,W
			sublw	.5
			btfsc	ZERO
			goto	clr5
			movf	TEMP2,W
			sublw	.6
			btfsc	ZERO
			goto	clr6
			goto	clr7
clr0:              bcf       INDF0, 7
              return
clr1:              bcf       INDF0, 6
              return
clr2:              bcf       INDF0, 5
              return
clr3:              bcf       INDF0, 4
              return
clr4:              bcf       INDF0, 3
              return
clr5:              bcf       INDF0, 2
              return
clr6:              bcf       INDF0, 1
              return
clr7:              bcf       INDF0, 0
              return
;===============================================================================
;GetWorkBit -- rk0..work7(rombit_idx) -> W
;===============================================================================
GetWorkBit:   call      SetupFSR         		; viita soovitud bitile
;              movlw     HIGH GetWorkBit
;              movwf     PCLATH
;              bcf		CARRY
;              rlcf       TEMP2, W         		; get bit position * 2 into w
;              addwf     TEMP2, W         		; compute w = bit offset * 3 (0, 3, 6 ...)
;              addwf     PCL, F           		; bump PC to jump to appropriate test
			  movf		TEMP2,W
			  sublw		.0
			  btfsc		ZERO
			  goto 		get7;0
			  movf		TEMP2,W
			  sublw		.1
			  btfsc		ZERO
			  goto 		get6;1
			  movf		TEMP2,W
			  sublw		.2
			  btfsc		ZERO
			  goto 		get5;2
			  movf		TEMP2,W
			  sublw		.3
			  btfsc		ZERO
			  goto 		get4;3
			  movf		TEMP2,W
			  sublw		.4
			  btfsc		ZERO
			  goto 		get3;4
			  movf		TEMP2,W
			  sublw		.5
			  btfsc		ZERO
			  goto 		get2;5
			  movf		TEMP2,W
			  sublw		.6
			  btfsc		ZERO
			  goto 		get1;6
			  goto 		get0;7
;Bit 0
get7:              btfss     INDF0, 7          		; Is bit set(1)
               retlw    d'0'             		; No, return 0
              retlw     d'1'             		; Yes, return 1
;Bit 1
              btfss     INDF0, 6         	 	; Is bit set(1)
get6:               retlw    d'0'             		; No, return 0
              retlw     d'1'             		; Yes, return 1
;Bit 2
get5:              btfss     INDF0, 5          		; Is bit set(1)
               retlw    d'0'             		; No, return 0
              retlw     d'1'             		; Yes, return 1
;Bit 3
get4:              btfss     INDF0, 4          		;Is bit set(1)
               retlw    d'0'             		; No, return 0
              retlw     d'1'             		; Yes, return 1
;Bit 4
get3:              btfss     INDF0, 3          		;Is bit set(1)
               retlw    d'0'             		; No, return 0
              retlw     d'1'             		; Yes, return 1
;Bit 5
get2:              btfss     INDF0, 2          		;Is bit set(1)
               retlw    d'0'             		; No, return 0
              retlw     d'1'             		; Yes, return 1
;Bit 6
get1:              btfss     INDF0, 1          		;Is bit set(1)
               retlw    d'0'             		; No, return 0
              retlw     d'1'             		; Yes, return 1
;Bit 7
get0:              btfss     INDF0, 0          		; Is bit set(1)
               retlw    d'0'             		; No, return 0
              retlw     d'1'             		; Yes, return 1
;===============================================================================
; SetupFSR -- Point FSR0L at byte within work area determined by rombit_idx
;   Entry:
;         rombit_idx contians bit number (one relative) we need to get to
;   Exit:
;         FSR0L - pointing at byte within work0..work7
;         TEMP1 - zero relative byte offset (0,1,..7)
;         TEMP2 - zero relative(0,1,..7) bit offset within byte
;===============================================================================
SetupFSR:                                 		; Setup FSR0L, TEMP1(byte offset), and TEMP2(bit offset)
              decf      rombit_idx, W    		; convert index to 0 relative
              sublw     d'63'            		; form compliment of rombit_idw 0-63->63-0
              movwf     TEMP1            		; and temp byte offset area
              andlw     b'00000111'      		; strip off byte offset leaving bit offset only
              movwf     TEMP2            		; and save into temp bit offset area
              
              bcf		CARRY            		; right justify the byte offset in TEMP1
              rrcf      TEMP1, F         		; by right shifting
              bcf		CARRY                   ; and stripping 3 LSB's 
              rrcf      TEMP1, F
              bcf		CARRY
              rrcf      TEMP1, F
;Get FSR0L pointing at appropriate byte in work area (work0, work1 ...)
              LFSR      .0,work0            	;Point FSR0L at beginning of work area
              movf      TEMP1,W            		;get byte offset (0 relative)
              addwf     FSR0L,F           		; point FSR0L straight at it
			  btfsc		CARRY 
			  incf		FSR0H,F
              return
;===============================================================================
; ******* reseti taimerite asjad *******
;===============================================================================
decrtmrs:		btfsc	reset1ena				; pingestamise viiteaeg juba l�bi ?
				goto	T1intr1end				; jah
				movf	reset1strttmrH,W		; kas etteantud aeg = 0 ?
				addlw	.0
				btfss	ZERO
				goto	rtmr1notnull
				movf	reset1strttmrL,W
				addlw	.0
				btfsc	ZERO
				goto	rtmr1z					; jah, viiteaeg oli = 0, siis ei ootagi
rtmr1notnull:	decfsz	reset1strttmrL			; ei, pingestamisej�rgne reset 1 viitetaimer alles k�ib
				goto	T1intr2
				movf	reset1strttmrH,W		; HIGH juba on  ?
				addlw	.0
				btfsc	ZERO
				goto	rtmr1z					; jah, siis aeg t�is
				decf	reset1strttmrH,F
			decf	reset1strttmrL,F
				goto	T1intr2
rtmr1z:			bsf		reset1ena				; reseti 1 generaator on n��d enableeritud
;				bsf		pank1
;				movf	Register277+.1,W		; lae side kadumise viiteaeg
;				bcf		pank1
;				movwf	reset1dlytmr
T1intr1end:		btfsc	reset1pulseon			; reseti 1 pulss juba aktiivne ?
				goto	T1intr1end1				; jah

				movf	reset1dlytmr,W			; kas side viiteaeg anti = 0 ?
				addlw	.0
				btfsc	ZERO
				goto	sv10					; jah, siis on aeg oodatud !

				decfsz	reset1dlytmr			; ootame kannatlikult, ehk side taastub
				goto	T1intr2					
sv10:;			bsf		pank1
				movf	Register277,W			; piisavalt InterNetita oldud: 
;				bcf		pank1
				movwf	reset1pulsetmr			; laeme pulsi kestuse
				addlw	.0
				btfsc	ZERO					; nulline kestus t�hendab, et pulssi ei tekitatagi !
				goto	R1zero
				bsf		reset1pulseon			; m�rgime �ra
;				bcf		reset1					; ja n��d annab saapaga ... !
			bcf		n_reset1					; inversioon
R1zero:;			bsf		pank1
				movf	Register277+.1,W		; taasta side kadumise viiteaeg
;				bcf		pank1
				movwf	reset1dlytmr
				goto	T1intr2					
T1intr1end1:	decfsz	reset1pulsetmr			; tixub pulsi kestust
				goto	T1intr2					
;				bsf		reset1					; aitab kah pexmisest...
			bsf		n_reset1					; inversioon
				bcf		reset1pulseon			; m�rgime �ra
;				bsf		pank1
				movf	Register277,W			; laeme uuesti pulsi kestuse
;				bcf		pank1
				movwf	reset1pulsetmr
			
T1intr2:		btfsc	reset2ena				; pingestamise viiteaeg juba l�bi ?
				goto	T1intr2end				; jah
				movf	reset2strttmrH,W		; kas etteantud aeg = 0 ?
				addlw	.0
				btfss	ZERO
				goto	rtmr2notnull
				movf	reset2strttmrL,W
				addlw	.0
				btfsc	ZERO
				goto	rtmr2z					; jah, viiteaeg oli = 0, siis ei ootagi
rtmr2notnull:	decfsz	reset2strttmrL			; ei, pingestamisej�rgne reset 2 viitetaimer alles k�ib
				return
				movf	reset2strttmrH,W		; HIGH juba on  ?
				addlw	.0
				btfsc	ZERO
				goto	rtmr2z					; jah, siis aeg t�is
				decf	reset2strttmrH,F
			decf	reset2strttmrL,F
				return
rtmr2z:			bsf		reset2ena				; reseti 2 generaator on n��d enableeritud
;				bsf		pank1
;				movf	Register279+.1,W		; lae side kadumise viiteaeg
;				bcf		pank1
;				movwf	reset2dlytmr
T1intr2end:		btfsc	reset2pulseon			; reseti 1 pulss juba aktiivne ?
				goto	T1intr2end1				; jah

				movf	reset2dlytmr,W			; kas side viiteaeg anti = 0 ?
				addlw	.0
				btfsc	ZERO
				goto	sv20					; jah, siis on aeg oodatud !

				decfsz	reset2dlytmr			; ootame kannatlikult, ehk side taastub
				return				

sv20:;			bsf		pank1
				movf	Register279,W			; piisavalt IntereNetita oldud: laeme pulsi kestuse
;				bcf		pank1
				movwf	reset2pulsetmr
				addlw	.0
				btfsc	ZERO					; nulline kestus t�hendab, et pulssi ei tekitatagi !
				goto	R2zero
				bsf		reset2pulseon			; m�rgime �ra
				bsf		PWRSW					; ja n��d annab saapaga ... !

R2zero:;			bsf		pank1
				movf	Register279+.1,W		; taasta side kadumise viiteaeg
;				bcf		pank1
				movwf	reset2dlytmr

				return					
T1intr2end1:	decfsz	reset2pulsetmr			; tixub pulsi kestust
				return					
				bcf		PWRSW					; aitab kah pexmisest...
				bcf		reset2pulseon			; m�rgime �ra
;				bsf		pank1
				movf	Register279,W			; laeme uuesti pulsi kestuse
;				bcf		pank1
				movwf	reset2pulsetmr
				return
;===============================================================================
; ******************* DSxxxx funktsioonid **************************************
;===============================================================================
;===============================================================================
; ******* loeb andurilt n�idu ja k�ivitab j�rgmise m��tmise *******
;===============================================================================
GetTemp:		decfsz	_10sekunditmr
				goto	GetTemp_end
				movlw	_10sekundiaeg
				movwf	_10sekunditmr
				movf    DS1820found,W			; kas on �ldse termoandureid ?
				addlw	.0
				btfss	ZERO
				goto	gettemp1				; on !
				movf    DS2438found,W			; aga neid teisi kive ?
				btfsc	ZERO
				goto	GetTemp_end				; ei ole midagi...
gettemp1:		bsf		temper
				goto	GetTemp_end				; m��dab siis kui aega on



GetT:			bsf		_1WSPU					; tugev toide maha 
				movlw	.16						; kui >16 siis kusitleme DS2438 kive
				cpfslt	DallasState
				goto	GetDS2438				; nii ongi !
				call	Reset_1wire				; liinile saabast
				call	saada_adr				; saada anduri aadress
				movlw	ReadScratch				; loe tulemust
				call	Sendbyte_1wireA
				decf	DallasState,W			; kalkuleerib anduri n�idu aadressi
				movwf	work0
				LFSR	.0,Dallas1
				bcf		CARRY
				rlcf	work0,W			
				bcf		CARRY
				addwf	FSR0L,F					; siin ta adre ongi
				btfsc	CARRY
				incf	FSR0H,F
				LFSR	.1,DallasChk			; leia kontrollbaidi aadress
				movf	work0,W
				addwf	FSR1L,F	
				btfsc	CARRY
				incf	FSR1H,F				
				call	Readbyte_1wire			; loe temperatuur (LSB)
				movwf	work7					; seivi sest tahan hiljem saata  MSB esimesena
				call	Readbyte_1wire			; MSB
				movwf	work0

				call	chk_temp				; kas 85C (v�i ei vastatud - kui MSB = 0xFF) ?
				btfss	CARRY
				goto	temp_nxt0				; ei ole 85C, kas vastus puudus �ldse ?
gett1:			btfsc	INDF1,.0				; esimene kord ?
				goto	Get_t_sec				; ei, teine kord
				movlw	0x03					; m�rgib, et oli paha n�it ja n��d on 2. kord
				movwf	INDF1
;				goto	Get_temp_dn				; ja seda n�itu ei usu
				call	Start_temp				; seda n�itu ei usu, k�ivita uus m��tmine
				bcf		_1WSPU					; tugev toide peale
				goto	GetTemp_end				; m��dame uuesti !
Get_t_sec:		btfsc	INDF1,.1				; 2. kord: kas eelmine oli samuti 85C ?
				goto	temp_nxt00				; jah aga kas �kki vastus puudub �ldse?
gett2:			clrf	INDF1					; eelmine n�it oli midagi muud, ei usu! markerid maha 
				movlw	0x10					; kirjutame n�iduks 4096 (0x1000)
				movwf	POSTINC0
				clrf	POSTINC0				
				goto	Get_temp_dn
temp_nxt0:		btfss	ZERO
				goto	temp_nxt1				; k�ik oli OK
				goto	gett1					; vastus puudus oopistykkis

temp_nxt00:		btfss	ZERO
				goto	temp_nxt1				; jah, siis usaldame
				goto	gett2					; vastus puudus, m�rgi �ra!
temp_nxt1:		clrf	INDF1					; markerid maha
;
; siia v�rdlus: kui >7d0 v�i F828 siis n�itu ei uuendata
temp_nxt:		movf	work0,W
				sublw	0x07
				btfss	CARRY
				goto	Get_temp_dn
				movf	work0,W
;
				movwf	POSTINC0
				movff	work7,POSTINC0			; meenutame LSB-d
Get_temp_dn:	call	Start_temp				; k�ivita uus m��tmine
				bcf		_1WSPU					; tugev toide peale 
				decf	DallasState,F			; 1 andur l�bi k�idud
				movf    DallasState,W			; k�ik ?
				addlw	.0
				btfss	ZERO
				goto	GetTemp_end				; eip !
				movf    DS2438found,W			; kas neid teisi kive ka on ?
				btfsc	ZERO
				goto	Get_temp_dn1			; ei - loeme vaid DS1820-d
				swapf    DS2438found,W			; DS2438 kivide hulk l�heb vanemaks nibleks
				addlw	.4						; noorem n�itab suhtlemise seisundimasina seisu. Igale kivile on 4 seisundit
				movwf	DallasState
				goto	GetTemp_end				; selguse m�ttes...

Get_temp_dn1:	movf    DS1820found,W			; alustame algusest
				andlw	0x0F					; tarbetu...
				movwf	DallasState
GetTemp_end:	return

chk_temp:		movf	work0,W					; Cy=1 kui n�it 0x550 ehk 85 C. Kui Z=1 siis vastus puudus
				sublw	0x05
				btfss	ZERO
				goto	chk_temp1				; kas ei vastatud (0xFF) ?
				movf	work7,W					; LSB
				sublw	0x50
				btfss	ZERO
				goto	temp_ok
				bcf		ZERO					; vastus oli
temp_bad:		bsf		CARRY					; aga kuidagi valelik
				return
temp_ok:		bcf		CARRY
				bcf		ZERO
				return
chk_temp1:		movf	work0,W
				sublw	0xFF
				btfss	ZERO
				goto	temp_ok
				bsf		ZERO					; m�rgime et ei vastatud
				goto	temp_bad
;---
dchk_temp:		movf	work0,W					; Kui Z=1 siis vastus puudus
				sublw	0xFF					; MSB
				btfss	ZERO
				goto	dtemp_ok				; on vist OK
				movf	work7,W					; LSB
				sublw	0xFF
				btfss	ZERO
				goto	dtemp_ok
dtemp_bad:		bsf		ZERO					; vastus puudus
				return
dtemp_ok:		bcf		ZERO
				return

; 4 k�ivita T
; 3 loe T, k�ivita Vdd
; 2 loe Vdd, l�lita Vadc, k�ivita Vadc
; 1 loe Vadc, ja I
GetDS2438:		movf	DallasState,W			; mida teha ?
				andlw	0x0F
				btfsc	ZERO
				goto	GetDS2438_nxt			; v�tta j�rgmine andur
				sublw	.1
				btfsc	ZERO
				goto	GetDS2438_s1			; state 1 ehk loe Vadc ja I ning mine seisu 0
				movf	DallasState,W
				andlw	0x0F
				sublw	.2
				btfsc	ZERO
				goto	GetDS2438_s2			; state 2 ehk loe Vdd ja l�lita Vadc k�lge ja k�ivita Vadc
				movf	DallasState,W
				andlw	0x0F
				sublw	.3
				btfsc	ZERO
				goto	GetDS2438_s3			; state 3 ehk loe T ja l�lita Vdd k�lge ja k�ivita Vdd
GetDS2438_s4:	call	Reset_1wire				; state 4 ehk k�ivita T m��tmine. Liinile saabast
				call	saada_adr				; saada anduri aadress
				movlw	ConvertT				
				call	Sendbyte_1wireA
				bcf		_1WSPU					; tugev toide peale
				decf	DallasState,F			; v�tab j�rgmise seisu
;
				movlw	T1resoL					; lae taimer 1 uuesti
				movwf	TMR1L
				movlw	T1resoH
				movwf	TMR1H

				return
GetDS2438_s3:	call	Reset_1wire				; seis 3: loe T, k�ivita Vdd
				call	saada_adr				; saada anduri aadress
				movlw	RecallMem				; kopeeri tulemused scratchpad'i
				call	Sendbyte_1wireA
				movlw	.0						; lk. 0
				call	Sendbyte_1wireA
				call	Reset_1wire				; saabast ja siis r��gib edasi
				call	saada_adr				; saada anduri aadress
				movlw	ReadScratch				; loe tulemust
				call	Sendbyte_1wireA
				movlw	.0						; lehek�lg 0
				call	Sendbyte_1wireA
				swapf	DallasState,W			; kalkuleerib anduri n�idu aadressi
				andlw	0x0F
				movwf	work0
				decf	work0,F
				LFSR	.0,DS2438_1
				bcf		CARRY
				rlcf	work0,F			
				rlcf	work0,F			
				rlcf	work0,W			
				bcf		CARRY
				addwf	FSR0L,F					
				btfsc	CARRY
				incf	FSR0H,F
				movf	DallasState,W
				andlw	0x0F
				movwf	work0
				bcf		CARRY
				rlcf	work0,W
				bcf		CARRY
				addwf	FSR0L,F					; siin ta adre ongi
				btfsc	CARRY
				incf	FSR0H,F
;--- temperatuuri puhul kontrollime kas vastus oli ja kui puudus, kirjutab n�idux 0x1000 ---
				LFSR	.1,DS2438Chk			; leia kontrollbaidi aadress
				swapf	DallasState,W
				andlw	0x0F
				movwf	work0
				decf	work0,W
				addwf	FSR1L,F	
				btfsc	CARRY
				incf	FSR1H,F				
				call	Readbyte_1wire			; loe staatus, unusta
				call	Readbyte_1wire			; loe temperatuur (LSB)
				movwf	work7					; seivi sest tahan hiljem saata  MSB esimesena
				call	Readbyte_1wire			; MSB
				movwf	work0
				call	dchk_temp				; Z=1 kui ei vastatud
				btfss	ZERO
				goto	dtemp_nxt				; k�ik oki-doki
dgett1:			btfsc	INDF1,.0				; esimene kord ?
				goto	dGet_t_sec				; ei, teine kord. N��d k�ll lajatab...
				movlw	0x01					; m�rgib, et oli paha n�it ja n��d on 2. kord
				movwf	INDF1
				goto	dGet_temp_dn			; ja seda n�itu ei usu (aga veel �le ei kirjuta !)
dGet_t_sec:		clrf	INDF1					; ei vastata: markerid maha 
				movlw	0x10					; ja kirjutame n�iduks 4096 (0x1000)
				movwf	POSTINC0
				movlw	0x00
				movwf	POSTINC0				
;	movlw	0x33
;	movwf	POSTINC0
;	movlw	0x44
;	movwf	POSTINC0
				goto	dGet_temp_dn
dtemp_nxt:		clrf	INDF1					; markerid maha
;	movlw	0x11
;	movwf	POSTINC0
;	movlw	0x22
;	movwf	POSTINC0
				movff	work0,POSTINC0			; salvesta n�it: MSB
				movff	work7,POSTINC0			; ja LSB
;--- temperatuuri puhul kontrollime kas vastus oli ja kui puudus, kirjutab n�idux 0x1000 ---
;				call	Readbyte_1wire			; loe staatus, unusta
;				call	Readbyte_1wire			; loe temperatuur (LSB)
;				movwf	work7					; seivi sest tahan hiljem saata  MSB esimesena
;				call	Readbyte_1wire			; MSB
;				movwf	POSTINC0
;				movff	work7,POSTINC0			; LSB
dGet_temp_dn:	call	Reset_1wire				; l�litame ADC Vdd k�lge
				call	saada_adr				; saada anduri aadress
				movlw	WrScr					; kirjuta scratchpad'i
				call	Sendbyte_1wireA
				movlw	.0						; lk. 0
				call	Sendbyte_1wireA
				movlw	_VddConnected			; l�lita ADC sisend Vdd k�lge
				call	Sendbyte_1wireA
				call	Reset_1wire				; stardime pinge m��tmise
				call	saada_adr				; saada anduri aadress
				movlw	ConvertV				
				call	Sendbyte_1wireA
				bcf		_1WSPU					; tugev toide peale
				decf	DallasState,F			; v�tab j�rgmise seisu	
				return
GetDS2438_s2:	call	Reset_1wire				; seis 3: loe Vdd, l�lita Vadc, k�ivita Vadc
				call	saada_adr				; saada anduri aadress
				movlw	RecallMem				; kopeeri tulemused scratchpad'i
				call	Sendbyte_1wireA
				movlw	.0						; lk. 0
				call	Sendbyte_1wireA
				call	Reset_1wire				; saabast ja siis r��gib edasi
				call	saada_adr				; saada anduri aadress
				movlw	ReadScratch				; loe tulemust
				call	Sendbyte_1wireA
				movlw	.0						; lehek�lg 0
				call	Sendbyte_1wireA
				swapf	DallasState,W			; kalkuleerib anduri n�idu aadressi
				andlw	0x0F
				movwf	work0
				decf	work0,F
				LFSR	.0,DS2438_1
				bcf		CARRY
				rlcf	work0,F			
				rlcf	work0,F			
				rlcf	work0,W			
				bcf		CARRY
				addwf	FSR0L,F					
				btfsc	CARRY
				incf	FSR0H,F
				decf	DallasState,W
				andlw	0x0F
				addlw	.1
				movwf	work0
				bcf		CARRY
				rlcf	work0,W
				bcf		CARRY
				addwf	FSR0L,F					; siin ta adre ongi
				btfsc	CARRY
				incf	FSR0H,F
				call	Readbyte_1wire			; loe staatus, unusta
				call	Readbyte_1wire			; loe temperatuur, unusta
				call	Readbyte_1wire			
				call	Readbyte_1wire			; loe pinge (LSB)
				movwf	work7					; seivi sest tahan hiljem saata  MSB esimesena
				call	Readbyte_1wire			; MSB
				movwf	POSTINC0
				movff	work7,POSTINC0			; LSB
				call	Reset_1wire				; l�litame ADC Vadc k�lge
				call	saada_adr				; saada anduri aadress
				movlw	WrScr					; kirjuta scratchpad'i
				call	Sendbyte_1wireA
				movlw	.0						; lk. 0
				call	Sendbyte_1wireA
				movlw	_VadcConnected			; l�lita ADC sisend Vadc k�lge
				call	Sendbyte_1wireA
				call	Reset_1wire				; stardime pinge m��tmise
				call	saada_adr				; saada anduri aadress
				movlw	ConvertV				
				call	Sendbyte_1wireA
				bcf		_1WSPU					; tugev toide peale
				decf	DallasState,F			; v�tab j�rgmise seisu	
				return
GetDS2438_s1:	call	Reset_1wire				; seis 1: loe Vadc, ja I
				call	saada_adr				; saada anduri aadress
				movlw	RecallMem				; kopeeri tulemused scratchpad'i
				call	Sendbyte_1wireA
				movlw	.0						; lk. 0
				call	Sendbyte_1wireA
				call	Reset_1wire				; saabast ja siis r��gib edasi
				call	saada_adr				; saada anduri aadress
				movlw	ReadScratch				; loe tulemust
				call	Sendbyte_1wireA
				movlw	.0						; lehek�lg 0
				call	Sendbyte_1wireA
				swapf	DallasState,W			; kalkuleerib anduri n�idu aadressi
				andlw	0x0F
				movwf	work0
				decf	work0,F
				LFSR	.0,DS2438_1
				bcf		CARRY
				rlcf	work0,F			
				rlcf	work0,F			
				rlcf	work0,W			
				bcf		CARRY
				addwf	FSR0L,F					
				btfsc	CARRY
				incf	FSR0H,F
				call	Readbyte_1wire			; loe staatus, unusta
				call	Readbyte_1wire			; loe temperatuur, unusta
				call	Readbyte_1wire			
				call	Readbyte_1wire			; loe pinge (LSB)
				movwf	work7					; seivi sest tahan hiljem saata  MSB esimesena
				call	Readbyte_1wire			; MSB
				movwf	POSTINC0
				movff	work7,POSTINC0			; LSB
				call	Readbyte_1wire			; loe vool (LSB)
				movwf	work7					; seivi sest tahan hiljem saata  MSB esimesena
				call	Readbyte_1wire			; MSB
				movwf	POSTINC0
				movff	work7,POSTINC0			; LSB
				bcf		_1WSPU					; tugev toide peale
				decf	DallasState,F			; v�tab j�rgmise seisu. Peaks olema 0 ehk alustame algusest
				return
GetDS2438_nxt:	swapf	DallasState,W			; kas on veel andureid?
				andlw	0x0F
				movwf	work0
				decf	work0,W
				addlw	.0
				btfsc	ZERO
				goto	Dallas_algusest			; eip, alustame uut ringi ja DS1820-ga
				movlw	.16						; adresseeri uus andur
				subwf	DallasState,F
				movf	DallasState,W
				andlw	0xF0
				addlw	.4						; alustame j�lle seisundist 4
				movwf	DallasState
				bcf		_1WSPU					; tugev toide peale 
				return
Dallas_algusest:movf	DS1820found,W			; alustame viimase DS1820-ga
				andlw	0x0F					; kas DS1820-d on �ldse ?
				addlw	.0
				btfss	ZERO
				goto	da1						; on ikka
				swapf	DS2438found,W			; eip, v�tame DS2438-d
				andlw	0xF0
				addlw	.4
				movwf	DallasState
				bcf		_1WSPU					; tugev toide peale 
				return
da1:			movwf	DallasState
				call	Start_temp				; k�ivita uus temp. m��tmine
				bcf		_1WSPU					; tugev toide peale 
				return
;===============================================================================
; ******************* saadab vastavalt DallasStatele sobiva aadressi ***********
;===============================================================================
saada_adr:;	bsf		Dir
;				movlw	'A'
;				call	SendCHAR
;				movlw	'D'
;				call	SendCHAR
;				movlw	'R'
;				call	SendCHAR
;	bcf		Dir
		movlw	MatchRom				; k�sk vaid konkreetsele andurile
				call	Sendbyte_1wireA
				movlw	.16						; kui >16 siis kusitleme DS2438 kive
				cpfsgt	DallasState
				goto	saada_adr_DS			; on DS1820 andurid	
				LFSR	.0,DS24381wa			; on DS2438 andurid
				swapf	DallasState,W
				andlw	0x0F
				movwf	work0
				decf	work0,F
				goto	saada_adr_1
saada_adr_DS:	decf	DallasState,W			; kalkuleerib anduri ID aadressi
				movwf	work0
				LFSR	.0,Dallas1wa
saada_adr_1:;	bcf		CARRY
			;	rlcf	work0,W			
			;	movwf	work1
			;	rlcf	work1,F			
			;	rlcf	work1,W
;				addlw .7
	
				movf	work0,W
				mullw	.8
				movf	PRODL,W
				bcf		CARRY			
				addwf	FSR0L,F
				btfsc	CARRY
				incf	FSR0H,F					

				movlw	.7
				bcf		CARRY			
				addwf	FSR0L,F
				btfsc	CARRY
				incf	FSR0H,F					; siin ta adre ongi

				movlw	.8						; seda on tervelt 8 baiti
				movwf	work3
saada_adr_loop:;	movf	INDF0,W
;	bsf		Dir
;			call	SendCHAR
;	bcf		Dir
				movf	POSTDEC0,W
				call	Sendbyte_1wireA			
				decfsz	work3
				goto	saada_adr_loop			; saadab k�ik ID 8 baiti
				return
;===============================================================================
; ******* k�ivitab temp. m��tmise *******
;===============================================================================
Start_temp:		call	Reset_1wire
				movlw	SkipRom					; uus m��tmine k�ima
				call	Sendbyte_1wireA
				movlw	ConvertT				
				call	Sendbyte_1wireA
				return
;===============================================================================
; ******* viited *******
;===============================================================================
wait:			movlw	0xFF
wait_x:	;		bsf		pank
				movwf	PR2
;				bcf		pank
			movlw	0x01					; T2 periood 1uS, seisma!
			movwf	T2CON
				clrf	TMR2
				bcf		PIR1,TMR2IF
				bsf		T2CON,TMR2ON
wait_1:			btfss	PIR1,TMR2IF
				goto	wait_1
				bcf		T2CON,TMR2ON
				bcf		PIR1,TMR2IF
				return
;===============================================================================
; ******* initsialiseerimine *******
;===============================================================================
init_analoog:	LFSR	.0,ch1sum
				movlw	Akanaleid
				movwf	meascount
init_a1:		movlw	m��tmisi
				movwf	POSTINC0				; selle kanali keskmistamiste arv
				clrf	POSTINC0				; tulemus
				clrf	POSTINC0
				movlw	MinInitialH
				movwf	POSTINC0				; Min
				movlw	MinInitialL
				movwf	POSTINC0
				movlw	MaxInitialH
				movwf	POSTINC0				; Max
				movlw	MaxInitialL
				movwf	POSTINC0
				decfsz	meascount
				goto	init_a1
				clrf	meascount				; jooksva kanalite number
				return		

init:
; **** WDT **** 
				clrf	WDTCON
				clrwdt                    		; WDT nullida 
; **** konf ****
				bsf		INTCON2,.7				; B-pullupid maha
;*** Taimer 0 *****
				movlw	B'01000011'				; 8 bitine, 1:16 prescaler T0-le, taimerina, seisma !
;				movlw	0x47					; 1:256 prescaler T0-le, seisma, 8-bitine
				movwf	T0CON					
				movlw	T0reso					; lae taimer 0 (katkestus iga 250 us tagant)
				movwf	TMR0L
				bcf		INTCON,TMR0IF
; **** komparaatorid ****
				BANKSEL CM1CON 
				movlw 	0x00					; analoog komparaatorid v�lja
				movwf 	CM1CON 
				movwf 	CM2CON 
				clrf	CVRCON					; tugipinge allikas OFF
;--- A/D muundi ---
				BANKSEL ADCON1 
				MOVLW 	B'00110000' 			; Selects the special trigger from the ECCP1,Vref=4,1V,A/D VREF-=0,Analog Negative Channel=GND
				MOVWF 	ADCON1 					
				BANKSEL ADCON2 
				MOVLW 	B'10111010' 			; right justified,-, 20 TAD, FOSC/32
				MOVWF 	ADCON2 					
				BANKSEL ADCON0 
				MOVLW 	chan1		 			; kanal 1
				MOVWF 	ADCON0 
;--- setup, pordid --------						; loe setup
				banksel WPUB     		 
				movlw	0x00					; WPU-d maha
				movwf	WPUB
				banksel PORTA    		 
				movlw	0x00					; port sellisesse l�hteseisu
				movwf 	PORTA 
				banksel TRISB     		 
				movlw	0xFF					; k�ik digisisendid ja sisenditeks
				movwf 	TRISB 
				banksel PORTB    		 
				movlw	0xFF					; port sellisesse l�hteseisu
				movwf 	PORTB 
				banksel TRISC     		 
				movlw	0xC4					; serial ja RdRxD on sisend, DIR ja muud v�ljunditeks (v.a. 1Wire data otc ja C3 mis oli enne reset2)
				movwf 	TRISC 
				banksel PORTC    		 
				movlw	0xC8;7					; port sellisesse l�hteseisu (n_reset1 teisipidi)
				movwf 	PORTC 
				banksel TRISD     		 
				movlw	0x00					; k�ik v�ljunditeks
				movwf 	TRISD 
				banksel PORTD    		 
				movlw	0x00					; port sellisesse l�hteseisu
				movwf 	PORTD 
				movlw	0x08					; port sellisesse l�hteseisu
				movwf 	PORTE 
;	bsf		PWRSW					
;********* Vahipeni ketti **************
				clrwdt                    		; WDT nullida 
				call	ajupesu					; m�lu killimine
				call	Read_Setup				; analoogkanali portide setup jms				
;--- USART -------------------- 				; USARTi h��lestus (SYNC = 0, BRGH = 0, BRG16 = 1)
; Register 273
; b0,1,2 - kiirus, default 19200 (010)
; b3 -debounce k�igile sisenditele, default  ON
; b4 - sticky bitt k�igile sisenditele, default OFF
; b5,6 - Wiegand B,A lubatus
; b7 - paarsuse bitt, (0= even parity), default EVEN
				call	setup_serial
				call	reset_ser1				; seriali reset, sidetaimeri reload
;--- p�hitaimeri reload --------				; lae s�steemi p�hitaimer
				movlw	T1resoL					; lae taimer 1 (katkestus iga 10 mS tagant)
				movwf	TMR1L
				movlw	T1resoH
				movwf	TMR1H
				movlw	0x31					; 1:8 prescaler, GO !
				movwf	T1CON					
;--- viitetaimeri reload --------				; lae Dallase viidete taimer
				movlw	0x01					; T2 periood 1uS, seisma!
				movwf	T2CON
				movlw	0xFF					; int peale 256 uS
				movwf	PR2
				bcf		PIE1,TMR2IE
				clrf	TMR2
				bcf		PIR1,TMR2IF
;;--- T4 viitetaimeri reload --------			; lae viidete taimer
;				movlw	0x0B					; T4 periood 1,1mS, seisma!
;				movwf	T4CON
;				movlw	0xFF					
;				movwf	PR4
;				bcf		PIE4,TMR4IE
;				clrf	TMR4
;				bcf		PIR4,TMR4IF
;--- pisike viide maha rahunemiseks ----
				movlw	.10						; rahuneme maha 1 s jooxul
				movwf	_10sekunditmr
relax:			movlw	0xFF
				movwf	sekunditmr
relax1:			call	wait					; 375 uS
				decfsz	sekunditmr
				goto	relax1
				decfsz	_10sekunditmr
				goto	relax
;--- lubame intsid -------------
				banksel	PIE1
				movlw	B'00100001'		 		; luba vaid ser. receive ja T1 intsid
				movwf	PIE1
				banksel	.0
				movf	PORTB,W				
				movlw	B'01001000'				; Perifeeria intsid lubada, samuti RBIE int
				movwf	INTCON
				movlw	B'00010000'				; katskestusi ei ole esinenud, RS232 porti ei puudu: read-only
				movwf	PIR1	
				banksel	IOCB
				movlw	0xFF
				movwf	IOCB					; lubame k�ik pordi B muutuse katckestused
				banksel	.0
				bcf		INTCON,T0IE
				bcf		INTCON,PEIE			
;--- Variaablid ----------------				; muu pudi
				movf	DinPort,W
				movwf	dinpress
				comf	DinPort,W
				movwf	Register1
				movlw	0x00
				movwf	dinsticky
				movlw	wiegandtime
				movwf	wiegandAtimer
				movwf	wiegandBtimer
				clrf	WAbitcount
				clrf	WBbitcount
				movlw	sekundiaeg
				movwf	sekunditmr
				movlw	0xFF
				movwf	lastinputs				; PORTB sisendite eelmine seis oli 0xFF			
				movlw	_10sekundiaeg
				movwf	_10sekunditmr
				movlw	.4						; 1ms loendi
				movwf	_1mscount
				LFSR	.1,Dallas1				; DS1820 kontrollbaidid n�idaku et andurid puuduvad (0x1000)
				movlw	.9
				movwf	DS1820found
ds1820loop:		movlw	0x10
				movwf	POSTINC1
				movlw	0x00
				movwf	POSTINC1
				decfsz	DS1820found
				goto	ds1820loop

				LFSR	.1,DS2438_1				; DS2438 kontrollbaidid n�idaku samuti et andurid puuduvad (0x1000)
				movlw	.9*.4
				movwf	DS1820found
ds2438loop:		movlw	0x10
				movwf	POSTINC1
				movlw	0x00
				movwf	POSTINC1
				decfsz	DS1820found
				goto	ds2438loop

;--- Dallase andurite avastamine ----------------
discovery:		call    Search_1Wire_init		; Dallase lugemine nulli
				bcf		nomoreds2438			; DS2438-d pole veel piisavalt leitud
				bcf		temper					; DS1820-d pole veel piisavalt leitud
				clrf	DS1820found
				clrf	DS2438found
				clrf	DallasState
				clrwdt                    		; WDT nullida 
_search_next:	call    Search_1Wire     		; k�se otsida              
				clrwdt                    		; WDT nullida 
            	btfss   return_value,0  		; Oli midagi ?
	            goto    _search_end     		; sitte mittagi...
				bcf		dallas					; kui dallas=1 siis DS2438
				LFSR	.1,work7				; 8. bait on family code. DS18B20=0x28, DS2438=0x26. Selle j�rgi m��rame m�luaadressi kuhu ID salvestada
				movf	INDF1,W
				sublw	DS1820ID
				btfss	ZERO
				bsf		dallas					; oli DS2438 !
				btfsc	dallas					; kas neid andureid on juba piisavalt ?
				goto	disc_2
				btfsc	temper					; kas DS1820-d juba k�llalt ?
				goto	disc_end				; jah, j�ta vahele
				goto	disc_3					; veel on ruumi
disc_2:			btfsc	nomoreds2438			; kas DS2438-d juba sitaks palju ?
				goto	disc_end				; jah, j�ta vahele
				
disc_3:			LFSR	.0,Dallas1wa			; m��rame �ige m�luala alguse
				btfsc	dallas
				LFSR	.0,DS24381wa			
				LFSR	.1,work0
				bcf		CARRY
				btfss	dallas
				goto	disc_0
				rlcf	DS2438found,W
				goto	disc_1
disc_0:			rlcf	DS1820found,W			
disc_1:			movwf	_RS485chk				; Seivime andurite tabelisse.
				rlcf	_RS485chk,F			
				rlcf	_RS485chk,W	
				bcf		CARRY
				addwf	FSR0L,F
				btfsc	CARRY
				incf	FSR0H,F
				LFSR	.1,work0
				movlw	.8						; seda on tervelt 8 baiti
				movwf	countL
dl_mv_loop:		movff	POSTINC1,POSTINC0
				decfsz	countL
				goto	dl_mv_loop
				btfss	dallas					; loendame �igeid andureid
				goto	dl_mv_1
				incf	DS2438found,F			; DS2438 juures
				movlw    MaxDS2438Sensors
				xorwf    DS2438found,W
				btfss    ZERO			        ; kas piir k�es ?
				goto	_search_next			; eip, v�ta nekst Dallase m�lakas
				bsf		nomoreds2438			; DS2438-d on leitud max kogus, rohkem enam ei luba
				goto	_search_next			; piisavalt aga otsime ikkagi l�puni

dl_mv_1:		incf	DS1820found,F			; loendame andureid
				movlw    MaxDS1820Sensors
				xorwf    DS1820found,W
				btfss    ZERO			        ; kas piir k�es ?
				goto	_search_next			; eip, v�ta nekst Dallase m�lakas
				bsf		temper					; DS1820-d on leitud max kogus, rohkem enam ei luba
				goto	_search_next			; piisavalt aga otsime ikkagi l�puni

disc_end:		btfss	temper					; DS1820-e arv t�is ?
				goto	_search_next			; eip, otci veel
				btfss	nomoreds2438			; aga DS2438 ?
				goto	_search_next			; eip, otci veel

disc_end1:		call	Start_temp				; k�ik andurid m��tma ja kohe!
				bcf		_1WSPU					; tugev toide peale 
				movf    DS1820found,W	
				andlw	0x0F					; tarbetu k�ll aga las olla...		
				movwf	DallasState
				movlw	sekundiaeg
				movwf	sekunditmr
				goto	_search_done			; l�petame jama...
_search_end:	movf    DS1820found,W			; enam ei leia kuid kas enne leiti miskit ?
				addlw	.0
				btfss	ZERO
				goto	disc_end1				; midagi leiti, stardime temp. m��tmised
				movf    DS2438found,W			; aga DS2438 ?
				addlw	.0
				btfsc	ZERO
				goto	_search_done			; midagi ei leitud
				bcf		_1WSPU					; tugev toide peale 
				swapf    DS2438found,W			; leidsime vaid DS2438 kivid
				andlw	0xF0					; tarbetu k�ll aga las olla...
				addlw	.4						; alustame seisust 4		
				movwf	DallasState
				movlw	sekundiaeg
				movwf	sekunditmr
_search_done:	clrwdt                    		; WDT nullida 
				clrf	Measflags

				movlw	0x04					; Capture igal langeval frondil
				banksel	CCP2CON
				movwf	CCP2CON
				banksel	PIE3
				bsf		PIE3,CCP2IE
				banksel	.0

				return
;________________________________________________________
ajupesu:		LFSR	.0,.0					; ajupesu
kill_mem:		clrf	POSTINC0
				movf	FSR0H,W
				sublw	0x0F
				btfss	ZERO
				goto	kill_mem
				clrf	RegX+.0
				clrf	RegX+.1
				return
;===============================================================================
; ***************************** EEPROM *****************************************
;===============================================================================
; **** seadme ID ****
 code_pack 0xF00000								; EEPROM'i sisu 
e_ADR:						db 0x01				; R274 modbussi aadress
e_IDH:						db 0x82				; R258 vidina unikaalne ID, bait 1
e_IDL:						db 0x01				; R259 vidina unikaalne ID, bait 2
e_PUa:						db 0x00				; pullup mask ehk sisuliselt v�ljundite seis stardil sest PUsid juhitakse v�ljunditega
e_PUd:						db 0x00				; R272H analoog,R272L digi: pullup mask ehk sisuliselt v�ljundite seis stardil sest PUsid juhitakse v�ljunditega
e_ser:						db 0x1A				; R273 seriali ja sisendite lugemise parameetrid: vaata allpool
e_IOD:						db 0x00				; R275 ANA-pordi (UIO) suund, 1= v�ljund, bitthaaval)
e_anadigi:					db 0x00;C0				; R271L analoogpordi seisund stardil - analoog v�i digi. 1= digi
e_devtype					db 0xF1				; R256
e_firmwarenr				db 0x02,0x0D		; R257H ja L: F/w HIGH ja LOW
e_reset1:					db 0x02,0x58,0x05,0x1E		; R276,R277 60 sekundit, 5 sekund, 30 sekundit
e_reset2:					db 0x02,0x58,0x05,0x64		; R278,R279 600 sekundit, 5 sekund, 64 sekundit
e_xor:						db 0xFF				; R271H sellega XORitakse ANA-pordi sisendit (et teha juhitav aktiivne HI/LO
e_ADC:						db 0x30				; R270 ADC Vref-i konf. Vaid bitid 3,4,5 m�juvad (Selects the special trigger from the ECCP1,Vref=Avdd,A/D VREF-=0,Analog Negative Channel=GND)
e_pwmperiod:				db 0x00,0x00		; R150 ehk PWMi periood
e_pwmconf:					db 0x00,0x00		; R151 ehk PWMi konf - kui bitt=1 siis see kanal on PWM
; serialport:
;------------
; b0,1,2 - kiirus, default 19200 (010)
; b3 - debounce k�igile sisenditele, default: ON
; b4 - sticky bitt k�igile sisenditele, default: OFF
; b5 - wiegand B lubatud (PORTB,4 ja 5)
; b6 - wiegand A lubatud (PORTB,6 ja 7)
; b7 - paarsuse bitt, (0= even parity), default EVEN
	end			
;******************************************************************************