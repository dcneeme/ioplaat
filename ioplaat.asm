;===============================================================================
; Universaalne IO-plaat modbus'i liinil. 8 digisisendit, 8 väljundit, 8 analoogsisendit
; Ftakt=11,0592 MHz.
; Side: 9600,n,8,2 modbus RTU
; 29.12.: lugemise, kirjutamise ja konfi käsud testitud, OK. Koopia siit !
; analoogtsükli parendamine, WR reg0 samuti. Koopia!
; 1.1.2013.: justkui toimiks. Koodikaitse ka peale. Neemele testimisele. Koopia !
; 4.2.: kolime regitrid 3. panka...
;===============================================================================
;=============================================================================== 
; 9.12.2012.: alustame...
; Register 0: binaarsed väljundid.
; MSB on DO mis on alati DO. Kirjutamisel muudab väljundi seisu (ei
; salvestata), lugemisel annab väljundi hetkeseisu. Kuidagi konfitav ei ole.
; LSB on UIO (ANA) osa. Kirjutamisel muudab vastava analoogkanali seisu
; kui ta on väljund (muidu ei teee midagi). Lugemisel annab samuti vastava
; analoogkanali seisu kui ta väljund. Kui DI siis annab samuti seisu
; arvestades debouncet ja sticky't. Kui pinn on analoogsisend, annab nulli.
;
; Register 1: binaarsed sisendid.
; MSB on DI mis on alati DI ja annab alati vastava sisendi seisu. Ainult
; loetav, ei ole konfitav. DI-l on debounce ja sticky reziim (kõigile
; korraga). Ei ole konfitav (s.t. debounce aeg)
; LSB on UIO (ANA) osa.Kui vastav analoogsisend on konfitud DI-na, siis
; annab selle seisu. Kui ei, siis vastav bitt=0. Kui konfitud väljundina,
; siis annab samuti hetke seisu ? Analoogpordi suund on konfitav registri
; 274 kaudu (1=väljund) bitthaaval. Pull-up'id konfitavad registri 273
; kaudu (DO seisu muudab ka!). Mõlemad registrid salvestatakse EEPROMi ja
; loetakse stardil ning rakendatakse kohe. Kui UIO konfitud DI-na,
; kehtivad debounce ja sticky reziimid.
;
; Registrid 2..9. A/D muundi tulemused. Ainult loetavad. Ei ole konfitavad
; (otseselt). Kui mõni kanal on DI või DO, siis annab nulli.
;
; Register adr 255. R-only, annab device type 0xF1
; Register adr 258. R-only, annab seerianumbri 1. osa (130 dec)
; Register adr 259. R-only, annab seerianumbri 2. osa (1..dec)
; Register adr 271. R/W-, annab analoog/digitaal omaduse. 1= digisisend, bitthaaval
; Register adr 272. R/W-, annab pull-up'ide seisu (ehk DO vaikimisi seisu), bitthaaval
; Register adr 273. R/W, seriali seaded nagu tabelis
; Register adr 274. R/W, seadme modbus aadress
; Register adr 275. R/W, ANA-pordi (UIO) suund, 1= väljund, bitthaaval)
;
; Serial
; -------
; b0,1,2 - kiirus, default 19200 (010)
; b3 -debounce kõigile sisenditele, default  ON
; b4 - sticky bitt kõigile sisenditele, default OFF
; b5,6 - ei kasuta
; b7 - paarsuse bitt, (0= even parity), default EVEN
;
; Peale 30-sekundist side puudumist või arusaamatuid käske läheb vaikimisi
; seriali parameetritele (19200, even)
;------------------- LISA - RESETID ------------------
;reset1:
; - aeg s alates toite pealetulekust, mis peab olema ületatud. naiteks 600s
; - reset impulsi kestus s, näiteks 1 s
; - ilma ühegi eduka modbus lugemiseta möödunud aeg s, mille ületamisel (koos esimese tingimuse arvestamisega) impulss antakse. näiteks 30 s.
;
;PWRSW:
; - aeg s alates toite tulekust, mis peab olema ületatud, näiteks 10 s
; - reset impulsi kestus s. näiteks 5 s
; - ilma ühegi eduka modbus lugemiseta möödunud aeg s, mille ületamisel (koos esimese tingimuse arvestamisega) impulss antakse. näiteks 10 s.
;
;sisuliselt on need 2 watchdogi täpselt ühesugused, ainult seadistusparameetrid on erinevad. 
; maksimaalselt on vaja 6 registrit konfimiseks, aga parameetrid 2 ja 3 mahuks mõlema watchdogi puhul vajadusel ka yhte registrisse (ära eri baitidesse).
;jah, need voiks olla sellised kirjutatavad/loetavad registrid, mis meelde jaetakse, ka toite kadumisel. aadress voiks olla
;teiste konfiregistrite (275 jne) ees voi jarel, kirjutamine nagu ikka (st tavaline, mitte 0045 sees).
;------------------------ LISA - WIEGAND --------------------
;mul ei ole muude pikkustega kui 26 kokkupuudet olnud. aga barix on oma x8 moodulis niimoodi teinud
;et koodi pikkus on eraldi registris 11 (aadress järelikult 10) kirjas. 
;wiegand kood registrites 12..16 (adr 11..15). saadud bitid nihutatakse registritesse paremalt 
;sisse alates reg12, viimase baidi vanemad bitid voivad valed/vanad olla, neid tuleb loetud 
; pikkuse abil maskida. jargmist wiegand koodi enne vastu ei voeta, kui reg 11 nullistatud, 
;
; aga kas selle nooremat baioti nullistab reg 12...16 lugemine voi poeab sinna ise kirjutama, 
; sellest ei saa ma aru. lugemisel nullistumine oleks loogilisem, nagu sticky bit puhul. 
;kusjuures lugeda tuleks koik need registrid, kus viimase koodi bitid sisaldusid. 
;tegelikult võib seda x8 peal katsetada, mul on see olemas. ega midagi hullu ei juhtu, 
;kui midagi teisiti teha, aga samasuste korral on mul lihtne enda ja barixi tehtud io seadmeid 
;segada ja nendega yhtemoodi suhelda.

;selles x8 moodulis on 1 wiegand võimalik, kuid barionetis 2. barionet on aga keerulise firmwarega 
;seade, milles wiegand kood on justkui virtuaalsest serial pordist loetav, kusjuures esimeses 
; baidis on bittide arv ja wiegandlugeja tunnus (0 voi 1). 
; x8 sobib aga hasti eeskujuks sellisele modbus i/o moodulile.
; 27.1.: resetid kodeeritud, peax kuidagi testima aga rauda eip ole mitte...
; 28.1.: Alustame Wiegandiga...Eelnevast tegin koopia.
;-------------------------- LISA - loendid igale DIN-ile -------------------------
; 3.2.: kõik debouncega, sticky't ei ole. Aadressid alates 400-st. chk_len toimimise loogikat ei tea, lubab lugeda ja kõik!
; 
;To check if WIEGAND data is available the register 11 has to be
;read. The lower Byte will contain the amount of Bits received.
;If it is 00h then no new data was received. The higher Byte
;contains a bitmap of found temperature sensors (see next page)
;--------- Wiegand --------------
;The Bits of the received WIEGAND data are shifted in (from
;right to left) into the Registers 12 to 16 (5x16=80 bits max)

; loendie ja wiegandi mustand valmis. Teha veel wiegandi timeoutide nullimine vastavaid registreid lugedes.
; 10.2.: wiegamd ja kood ise kah justqi töötax - Neemele testimisele.
; 13.2.: sisendid aktiivse madala nivooga - muudatus registrites näitamises. Reset1 polaarsus vastupidiseks.
; 16.2.: kõik nagu toimib. Koopia! Hakkame tegema termomeetreid. Discovery töötab !!! Teeks coopia ?
; viga: DIN ei tööta kui loetakse 10 reg. korraga !?
; 17.2.: vead parandatud, loeb ID-sid ja temp. näitusid ka aga veel ei mõõda. 
; wiegandi ootepaus nüüd 30ms.Saadame termomeetritele mõõtmise käsu. Koopia siit!
; mõõdab termomeetritega aga näit on FF,FF !? CurrentDallas=bits, muidu kuidagi sai solgitud...
; töötab !!! Koopia kah (ver.10)
; 18.2.: confi puhul ei kirjutata setupi mällu ja serial porti, sea writeit ja silu veica ~!
; 19.2.: save_setup ja serint parandet. Tunneb ka broadcast aadressi! PWRSW paralleeli reset2-ga!
; 20.2.: Wiegandi puhul keelan seriali katckestuse, tagasi wiegandi timeoutiga.
; 22.2.: read_setup'i parandamine, reg 666 access teeb nüüd reseti.
; 23.2.: wiegandi registrite nullimine ja side lubamine paremaks. Koopia ! Edasi teeme  käskude täitmise väljaspool katckestusi...
; Registertemp'ide optimeerimine, käsud täidetakse väljaspool intse
; 24.2.: "rot" parandus, A-wiegandi parandus. Käsud väljaspool intse - töötab aga kaarti loeb ajuti ikkagi imelikult (bitid kaovad). Testida!
; 27.2.: seriali kiiruse paranduse parandus, 3 väikest lisa. e_XOR peakx modb_write ja setupi puhul ka tulemuse enne porti kirjutamist xor-ima !
; 3.3.: write vastas vale reg. aadressiga. Veel mõned fiksed. Vist nüüd ok.
; 10.3.: register 767 teeb reseti, Enne oli 666 aga see jäi Dallase reg-te alla.
; 12.3.: REg-te 271, 275 ja 0,1 toimimise täpsustamine. Koopia senisest.
;saadan tapsustavat infot ANA kaitumise kohta. anomaalia on kuidagi seoses reg 271 sisuga, millel ei tohiks tegelikult valjundiga pistmist olla. see register peaks mojutama ainult sisendina tootava ANA kaitumist, valides kas ai voi di moodi (ja di inversiooni).

;igatahes kui reg271 on 00FF, siis on voimalik registrisse 0 00FF kirjutamisega ka ANA noorimat poolbaiti korgeks ajada

;kui reg271 on 0000, siis aktiviseerub ainult vanem poolbait registrisse 0 00FF kirjutamisega.

;koige huvitavam on aga see, et kui reg 271 sisu muuta, ykskoik mida sinna sisse kirjutades, ilmub valjundi reziimis olevale ANA pordile alati kood AA!

;kirjutades registrisse 0 koodi 0000, saab selle valjundi kaotada. kirjutades nyyd registrisse 0 kood 00FF, saame tegelikuks valjundiks kas FF voi F0 soltuvalt sellest, kas reg 271 sisu on 00FF voi 0000.


;ja valjundina tootava ANA pordi lugemisel (reg0 LSB) saab alati 0 - et koik teadaolevad vead siinjuures korraga yles loetud oleks.

; 16.3.: pordi setup uuesti, reset2 ära (highZ), resetid veidi muudetud. No proovib nüüd....
; 18.3. resettide fiks, loendis viimase kanali kontroll ära - loendab kohe.
; 19.3.: Din2 biti lugemise viga: Din_2sethi juures oli bcf pank1 aesmel bsf pank1 !!!
; 28.3.: reset1 inversioonibitt PORTC,.3 lisatud.
; 6.4.: reload_side IGA baidi vastuvõtul ! Muudatused - CHG - Väidetalt tekib ajuti vastuse kadumine - uurida mispärast.
; 6.4.: side viga fiksed (vt. main juurest). Termo tõttu tekivad ajuti ikkagi viited. Vaja muuta termo algoritmi: katkestusest välja või state machine.
; 11.4.: termomeetreid pollib nüüd iga 10s tagant.
; 17.4.: lisa - peale Reset2 pulsi lõppu teeb endale reseti.
; 24.4.: lisatud watchdog ja firmware nr (Register 257). Reg. 256 on nüüd dev. type
; 26.4.: WDT-d tuli discovery käigus palju haamriga pähe taguda, muidu hakkas vastu. Tehtud !
; 27.4.: reg. kirjutamine oli 1 võrra nihkes, fiksed.
; 30.4.: loendite ja temp. lugemise fiks. Nüüd loeb ilma vastuste kaota. TEstima!
; 3.5.: lisasin temp. näidu kontrolli aga siis jooxis kokku: clrwdt + GIE keelata GetT ajaks. Ehk nüüd ... ?
; 3.5.: EI aga .. nüüd küll. Ma vihkan 16-seeria PICe... :)
; 12.5.: RCIE ja RCIF olid valedes registrites; reload_side ringi tõstet. Ehk nüüd parem (kippus segama teiste juttu,samuti termomeetrite kokkujooxmine)
; 17.5.: lisatud BOR.
; 3.6.: 32-bitine uptime loendi reg. 498...499 (1F2, 1F3) F/W ver. 0.4
; 26.12.: Register 272 ei mõjunud väljundite seisule - võtab uuest versioonist vastava koodi üle. Jääb pooleli...
; 28.12.: read_setup'is oli vaja enne porti kirjutamist teha banksel 0. Ver.: 0.5
;================================================================
;*********************** Raudvärk **********************************************
;*** Side ***
#define Dir				PORTC,5					; väljundpuhvri juhtimine, HIGH=rcv, LOW=trnsmt
#define TxD				PORTC,6					; seriali otc
#define RxD				PORTC,7					; seriali otc
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

;*** väljundid ***
#define DoutPort		PORTD
#define Dout0			PORTD,.0				; digiväljund 1
#define Dout1			PORTD,.1				; digiväljund 2
#define Dout2			PORTD,.2				; digiväljund 3
#define Dout3			PORTD,.3				; digiväljund 4
#define Dout4			PORTD,.4				; digiväljund 5
#define Dout5			PORTD,.5				; digiväljund 6
#define Dout6			PORTD,.6				; digiväljund 7
#define Dout7			PORTD,.7				; digiväljund 8

;*** 1-Wire ***
#define	_1WSPU			PORTC,.0				; 0 aktiveerib tugeva pull-up'i 1-wire liinil
#define	_1WDAT			PORTC,.1				; 1-Wire dataotc
#define	TRIS_1WDAT		TRISC,.1				; 1-Wire data TRIS-pinn
; *** resetid ***
#define	reset1			PORTC,.2				; 3,3V reseti otc
#define	n_reset1		PORTC,.3				; 3,3V reseti inversioon-otc

#define PWRSW			PORTC,.4				; mobla toitelüliti
;*** Wiegand ***
#define D0A				PORTB,.7				; A (0)-lugeja, siis digi-in ei funksi !
#define D1A				PORTB,.6
#define D0B				PORTB,.5				; B (1)-lugeja
#define D1B				PORTB,.4

;#define test			PORTC,.4
; ********************** constantsid *******************************************
;*** rauda puutuv **
;*** A/D muundi kanalid ***
#define chan1			0x81					; ADC kanal 0
#define chan2			0x85
#define chan3			0x89
#define chan4			0x8D
#define chan5			0x91
#define chan6			0x95
#define chan7			0x99
#define chan8			0x9D
#define	sampletime		0xFF
;****** debounce ***
#define debtime			.1						; 10ms debounce aega igale sisendile
;****** side *******
#define RSBufLen			.79;.53-.12				; saate-/vastuvõtu puhver
;****** käsud *****
#define modbus_rd			0x03				; käsk: loe registrit
#define modbus_wr			0x06				; käsk: kirjuta registrisse
#define modbus_wrmulti		0x10				; käsk: kirjuta mitu registrit korraga
#define modbus_cnf			0x45				; käsk: kirjuta konfiregistrisse
#define IllFunc				0x01				; veakood: keelatud käsk
#define IllDAdr				0x02				; veakood: vale registri aadress
#define maxregisters		.104;0				; niimitu 2-baidist registrit seadmel on
#define workregisters		.19;0					; niimitu tööregistrit on, muud on kõrgema adrega ja süsteemi setupi jaoks
#define sidetime			.30					; 30 s möödudes kui ei laeku arusaadavaid käsk, võetakse default sideparameetrid
;****** taimerid, ajad ***
#define	serialtime			0x06				; 30 ms käsu laekumise aega alates 1. baidist
#define sekundiaeg			.100
#define _10sekundiaeg		.10
#define	T1resoL				0x7F				; 10ms aeg @ 11.0592 MHz
#define	T1resoH				0xF2					
#define senstime			0x02;1				; 5 ms debounce aega
#define defaultserial		0x0A
#define	wiegandtime			.3;10					; 100 ms max ooteaega (viimasest pulsist loetuna)
; b0,1,2 - kiirus, default 19200 (010)
; b3 -debounce kõigile sisenditele, default  ON
; b4 - sticky bitt kõigile sisenditele, default OFF
; b5,6 - ei kasuta
; b7 - paarsuse bitt, (0= even parity), default EVEN
;****** Dallase kräpp *******
#define MaxDallasSensors	.9					; rohkem ei mahu mällu ää...
#define	ConvertT		0x44					; käsk: mõõda
#define	SkipRom			0xCC					; käsk: ID-ROMi lugemine vahele 
#define	MatchRom		0x55					; käsk: loe vaid konkreetset andurit
#define	ReadScratch		0xBE					; käsk: loe temp. ja muu

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

#define reset1ena			flags,.0			; reset1 aeg täis tiksunud (peale pingestamist) ja nüüd saab pulssi anda kui vaja)
#define reset2ena			flags,.1			; reset2 aeg täis tiksunud (peale pingestamist) ja nüüd saab pulssi anda kui vaja)
#define reset1pulseon		flags,.2			; reseti 1 pulsi kestuse taimer käib
#define reset2pulseon		flags,.3			; reseti 2 pulsi kestuse taimer käib
#define wiegandAto			flags,.4		
#define wiegandAtmron		flags,.5		
#define	wiegandBto			flags,.6
#define wiegandBtmron		flags,.7

#define reg10				flags1,.0			; loeti registrit 10: nulli ta sisu (wiegandi bitikaunt)
#define dallas				flags1,.1
#define temper				flags1,.2
#define cmd_ok				flags1,.3			; saadi käsk modbusilt
;;**** 1-wire ****
#define CurrentDallas		bits
;=============================================================================== 
;*********************** Raudvärk **********************************************
	include "P16F887.inc"  	
	__CONFIG    _CONFIG1, _FCMEN_OFF & _IESO_OFF & _LVP_OFF & _HS_OSC & _PWRTE_ON &  _MCLRE_ON & _WDT_ON & _CP_ON & _CPD_ON & _BOR_ON
	__CONFIG    _CONFIG2, _WRT_OFF & _BOR40V
				errorlevel -302					; Compaileri vingumised ära
				errorlevel -305
				errorlevel -306 

; ************** Mälu jaotus **********	    	
; ===== mälupank 1 =====
				cblock 0x20
					sidetaimer					; mõõdab 30s side kadumise aega
					sekunditmr
					lastinputs					; PORTB sisendite eelmine seis
					flags
					flags1
					sampledly					; mõõtmise Taqu viide, ka serpartemp'i asemel
; *** resettide taimerid ****
					reset1strttmrH
					reset1strttmrL
					reset1pulsetmr
					reset1dlytmr
					reset2strttmrH
					reset2strttmrL
					reset2pulsetmr
					reset2dlytmr
; *** käsu parseldamine jne ***
					countH
					countL						; ka Dallase otsimise rutiinis !
					adrtemp
; *** serial ***
					Char 			          	; vastuvõetud sümbol, saatmisel sendtemp'i asemel
					bytecnt
					Flags	
					serialtimer					; seriali taimer (40mS ja siis VIGA kui käsk ikka poolik)
; *** CHK arvutus ***
					mb_del1
					mb_temp1
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
					source
					destination
					datatemp
; *** Registrite kirjutamise abi ***
					Registertemp1
					Registertemp2
					Registertemp3
					Registertemp4
					Registertemp5
;					Registertemp6			
					Registertemp7				; ajutine register modb_writes ja setup_pordis
; *** loendite stuff ***
					muutus
					senstmrs
; *** Wiegand ***
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
					OneWireByte
					TEMP0
					TEMP1
					TEMP2
; Work areas
					rombit_idx
					bits
					devicesfound				; leitud Dallase andurite arv
					curr_discrep				; 0x70
					last_discrep
					done_flag
					return_value
; One-Wire work area
					work0                       ; CRC (8 bits)
					work1                       ; Serial # (48 bits) 
					work2                       ; " 				
					work3                       ; "
					work4                       ; "						
					work5                       ; "
					work6                       ; "
					work7                       ; Family Code (8 bits) (7A)

					_10sekunditmr				; 0x7B
 				endc

; *** konteksti seivimine ***
				cblock	0x7D
					W_Temp						; konteksti seivimine katckestuse ajaks
					S_Temp	
					FSRtmp	
 				endc
; ===== mälupank 2 =====
				cblock 0xA0						; kuni 0xEF
					Puhver:RSBufLen		        ; seriali puhver, kuni 79 baiti 
				endc
; ===== mälupank 3 =====
				cblock 0x110					; 110-16F	
; *** registrid ***
					Register0:.2				; digiväljundid (MSB) ja UIO (LSB), r/w	
					Register1:.2				; digisisendid (MSB) ja UIO (LSB), r	
					Register2:.2				; analoogsisendid ADC chan 1 (0) tulemus, r
					Register3:.2
					Register4:.2
					Register5:.2
					Register6:.2
					Register7:.2
					Register8:.2
					Register9:.2
					Register10:.2				; HIGH - Wiegand A bittide hulk, LOW - B kohta
					Register11:.2				; Wiegand A bitijada
					Register12:.2				
					Register13:.2				
					Register14:.2			
					Register15:.2				; Wiegand B bitijada
					Register16:.2				
					Register17:.2			
					Register18:.2			
; *** setup ****
					Register256:.2				; r, device type (0xF1)
					Register257:.2				; r, firmware nr HIGH ja LOW
					Register258:.2				; r, serial, 1. osa (130 dec)
					Register259:.2				; r, serial 2.osa (1...dec)
					Register271:.2				; r/w, UIO analoog/digitaalseis, 1=digisisend
					Register272:.2				; r/w,  pull-upp'ide seis ehk DO seis stardil, 1= PU sees
					Register273:.2				; r/w, seriali parameetrid
					Register274:.2				; r/w,  modbus'i aadress
					Register275:.2				; analoogpordi UIO suund, 1= väljund, 0= sisend
					Register276:.2				; r/w reset 1 toite pealetulemise kaitse aeg
					Register277:.2				; r/w reset 1 pulsi kestus (MSB) ja rakendumise aeg (LSB)
					Register278:.2				; r/w reset 2 toite pealetulemise kaitse aeg
					Register279:.2				; r/w reset 2 pulsi kestus (MSB) ja rakendumise aeg (LSB)
; *** loendid ***
					Loendi1:.4					; DIN 1 pulsside loendi
					Loendi2:.4
					Loendi3:.4
					Loendi4:.4
					Loendi5:.4
					Loendi6:.4
					Loendi7:.4
					Loendi8:.4					; kuni 0x16F.
				endc
; ===== mälupank 4 =====
; *** Dallase kräpp ;) ***
				cblock 0x190					; 190-1EF
					Dallas1:.2					; Dallase termomeetrite näidud (Reg. 600 -> 47)
					Dallas2:.2					; 601 -> 48
					Dallas3:.2					; 602 -> 49
					Dallas4:.2					; 603 -> 50
					Dallas5:.2					; 604 -> 51
					Dallas6:.2					; 605 -> 52
					Dallas7:.2					; 606 -> 53
					Dallas8:.2					; 607 -> 54
					Dallas9:.2					; 608 -> 55
					Dallas1wa:.8				; Dallase termomeetrite ID-d (1A2: Reg. 650, 651 -> 56,57 dec.)
					Dallas2wa:.8				; 1A6: 652, 653 -> 58, 59
					Dallas3wa:.8				; 1AA: 654, 655 -> 60, 61
					Dallas4wa:.8				; 62,63
					Dallas5wa:.8				; 64,65
					Dallas6wa:.8				; 66, 67
					Dallas7wa:.8				; 68, 69
					Dallas8wa:.8				; 6A, 6B
					Dallas9wa:.8				; 6C,6D - 0x1EA (jääb 5 baiti)		
; *** uptime loendi ***
					LoendiUP:.4					; 6E, 6F
									endc
 ; ************* asjad puhvris peale käsu vastuvõttu ****************	    	
#define			m_adr			Puhver+.0		; slave aadress
#define			m_cmd			Puhver+.1		; käsukood
#define			m_radrH			Puhver+.2		; loetava/kirjutatava registri aadress HIGH
#define			m_radrL			Puhver+.3		; loetava/kirjutatava registri aadress LOW
#define			n_regH			Puhver+.4		; loetavate/kirjutatavate registrite arv HIGH
#define			n_regL			Puhver+.5		; loetavate/kirjutatavate registrite arv LOW
; **** lipukesed ****
#define 		SerialTimerOn 	Flags,.0		; seriali vastuvõtu taimer s/v
#define			cmd10			Flags,.1		
#define			SerialTimeOut 	Flags,.2
#define 		readonly		Flags,.3		; kui register on read-only
#define 		sidetmron		Flags,.4
#define			writeit			Flags,.5
#define			clrsticky		Flags,.6		; kui loeti registrit 1 suvalises kombinatsioonis teistega
#define			reg0			Flags,.7

#define 		in_sticky		Registertemp2,.4
#define 		in_debounce		Registertemp2,.3
; **** serial port ****

; **** Prose lipukesed ****
#define CARRY           	STATUS,C 
#define ZERO            	STATUS,Z 
#define pank            	STATUS,RP0
#define pank1            	STATUS,RP1
; *****************************	    	
 
				org	0x000
   				goto	main
;===============================================================================
;*********************** katckestused ******************************************
;===============================================================================
				org 0x0004
_Push:
	    		movwf   W_Temp           		; Seivi kontekst        
				swapf   STATUS,W         
				movwf   S_Temp   
				movf	FSR,W
				movwf	FSRtmp 
				bcf		STATUS,IRP
				banksel	.0
				btfsc	INTCON,RBIF				; kas mingi Wiegandi daataots või digisisend (loendi) liigutas ennast?
				call	Loendid					; jepp, vaata kumb oli
Int1:			btfsc	PIR1,TMR1IF				; Taimer 1'e katckestus?
				goto	T1int
Int_2:			btfsc	PIR1,RCIF				; Seriali katckestus?
				call	SerInt					
_Pop:			movf	FSRtmp,W			
				movwf	FSR 					; taasta FSR
				swapf   S_Temp,W         
				movwf   STATUS           		; taasta STATUS         
				swapf   W_Temp,F         
				swapf   W_Temp,W         		; Taasta W         
				retfie                   	
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
				bsf		lastinputs,.0			; oli tõusev front, updteerime sisendite eelmist seisu
				goto	Andur_end				; ja vaatame järgmisi sisendeid
counter1_1:		bcf		lastinputs,.0			; updteerime sisendite eelmist seisu
				btfss	sens1tmron				; JUBA DEBOUBCEME ?
				goto	counter1_2				; ei veel, hakkab pihta
				goto	Andur_end				; jah, vaatame järgmisi sisendeid
counter1_2:		bsf		sens1tmron 				; taimer käima
				goto	Andur_end				; ja kõik !
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
; *** Loendi 5 või Wiegand B biti 0 sisend ***
counter5:; b5 - wiegand B lubatud (PORTB,4 ja 5)
; b6 - wiegand A lubatud (PORTB,6 ja 7)
				btfsc	Registertemp2,.5
				goto	Wiegand_Blow			; oli pulss ja Wiegand lubatud => töötleme Wiegandi järgi
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
; *** Loendi 6 või Wiegand B biti 1 sisend ***
counter6:		btfsc	Registertemp2,.5
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
; *** Loendi 7 või Wiegand A biti 0 sisend ***
counter7:		btfsc	Registertemp2,.6
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
; *** Loendi 7 või Wiegand A biti 1 sisend ***
counter8:		btfsc	Registertemp2,.6
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
Andur_end1:		bcf		INTCON,RBIF				; ei, siis võtame katkestuse nõude ka maha. Vastasel juhul tulex järgmisel korral tagasi ja analuusiks veel 
				return
;===============================================================================
; ******* PORTB sisendite (Wiegand) INT *******
;===============================================================================
Wiegand_Ahigh:	bsf		pank
				bcf		PIE1,RCIE
				bcf		pank
				btfsc	dataport,.7				; loeme vaid - fronti
				goto	wieg_endAH				; oli + front
				bcf		lastinputs,.7			; updateeri sisendite seisu
				btfsc	wiegandAto				; kui timeout siis on pakett juba käes ja loeme edasi vaid siis kui master on data ära lugenud
				goto	Andur_end				; minema siit !
				call	wieg_prepA				; + front, käivita paketi lõpu taimer
				call	rotH					; nihuta biti 1 väärtus registrisse
				incf	WAbitcount,F			; loendame bitte
				goto	Andur_end				; aitab !
Wiegand_Alow:	bsf		pank
				bcf		PIE1,RCIE
				bcf		pank
				btfsc	dataport,.6				; loeme vaid - fronti
				goto	wieg_endAL				; oli + front
				bcf		lastinputs,.6;7			; updateeri sisendite seisu
				btfsc	wiegandAto				; kui timeout siis on pakett juba käes ja loeme edasi vaid siis kui master on data ära lugenud
				goto	Andur_end				; minema siit !
				call	wieg_prepA				; + front, käivita paketi lõpu taimer
				call	rotL					; nihuta biti 0 väärtus registrisse
				incf	WAbitcount,F
				goto	Andur_end				; aitab !
Wiegand_Bhigh:	bsf		pank
				bcf		PIE1,RCIE
				bcf		pank
				btfsc	dataport,.5				; loeme vaid - fronti
				goto	wieg_endBH				; oli + front
				bcf		lastinputs,.5			; updateeri sisendite seisu
				btfsc	wiegandBto				; kui timeout siis on pakett juba käes ja loeme edasi vaid siis kui master on data ära lugenud
				goto	Andur_end				; minema siit !
				call	wieg_prepB				; + front, käivita paketi lõpu taimer
				call	rotH					; nihuta biti 1 väärtus registrisse
				incf	WBbitcount,F
				goto	Andur_end				; aitab !

Wiegand_Blow:	bsf		pank
				bcf		PIE1,RCIE
				bcf		pank
				btfsc	dataport,.4				; loeme vaid - fronti
				goto	wieg_endBL				; oli + front
				bcf		lastinputs,.4			; updateeri sisendite seisu
				btfsc	wiegandBto				; kui timeout siis on pakett juba käes ja loeme edasi vaid siis kui master on data ära lugenud
				goto	Andur_end				; minema siit !
				call	wieg_prepB				; + front, käivita paketi lõpu taimer
				call	rotL					; nihuta biti 1 väärtus registrisse
				incf	WBbitcount,F
				goto	Andur_end				; aitab !
;===============================================================================
wieg_prepA:		movlw	wiegandtime
				movwf	wiegandAtimer
				bsf		wiegandAtmron			; taimer paketi kestust valvama
				movlw	Register11+.7			; jooxva biti salvestamiseks vajaliku registri aadress
				movwf	FSR						; kirjutame sinna kaudselt
				return
wieg_prepB:		movlw	wiegandtime
				movwf	wiegandBtimer
				bsf		wiegandBtmron			; taimer paketi kestust valvama
				movlw	Register15+.7			; jooxva biti salvestamiseks vajaliku registri aadress
				movwf	FSR						; kirjutame sinna kaudselt
				return
;===============================================================================
rotH:			bsf		STATUS,IRP				; 3. mäluplokk !
				bsf		CARRY
				goto	rot
rotL:			bsf		STATUS,IRP
				bcf		CARRY
rot:			rlf		INDF,F
				decf	FSR,F
				rlf		INDF,F
				decf	FSR,F
				rlf		INDF,F
				decf	FSR,F
				rlf		INDF,F
				decf	FSR,F
				rlf		INDF,F
				decf	FSR,F
				rlf		INDF,F
				decf	FSR,F
				rlf		INDF,F
				decf	FSR,F
				rlf		INDF,F
				decf	FSR,F

				bsf		STATUS,IRP				; tagasi maa peale...
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
;				banksel	PIR1
				bcf		PIR1,RCIF				; katkestuse nõue maha
;				banksel	RCSTA
				movf	RCSTA,W					; oli viga?
				andlw	.6						; Viga vastuvõtul? Maskeeri tarbetud staatuse bitid
;				banksel .0
				btfss	ZERO
				goto	reset_ser				; oli, alusta uuesti
modbus_rcv:		bsf		SerialTimerOn	
				banksel	RCREG
				movf	RCREG,W
				banksel .0
				movwf	Char
				movf	bytecnt,W				; kontrolli puhvri piire
				sublw	RSBufLen-.1
				btfss	CARRY
				goto	reset_ser				; liiga pikk mess või miskit sassis, reset!
				movlw	Puhver					; arvutame baidi salvestamise koha vastuvõtupuhvris
				movwf	FSR						; puhvri pointer
				movf	bytecnt,W
				addwf	FSR,F
				movf	Char,W
				movwf	INDF					; salvesta bait
				incf	bytecnt,F
				movf	bytecnt,W				; mitmes bait oli
				sublw	.1
				btfsc	ZERO
				goto	modb_r0					; esimene
				movf	bytecnt,W				; äkki oli teine (käsk) ?
				sublw	.2
				btfss	ZERO
				goto	modb_r1					; ei, päris mitmes oli...
				movf	Char,W
;				movwf	mbCmnd					; seivib igax pettex
				sublw	modbus_cnf;modbus_wrmulti
				btfss	ZERO
				goto	modb_r1					; ei, siis ei näpi
				movlw	.12;7
				movwf	countL
				goto	RREnd1					; jääb kuuldele...

modb_r0:;		movf	Char,W					; 1. bait on aadress. Kas 0x00 ehk broadcast ?
;;				movwf	mbAdr					; seivime
;				addlw	.0
;				btfsc	ZERO
;				goto	modb_r12				; on broadcast
;
;				movf	Char,W					; Äkki käsk 0x10 ?
;				sublw	modbus_wrmulti
;				btfss	ZERO
;				goto	modb_r1					; ei, siis ei näpi - tavaline riid
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
;				movwf	mbnregH					; seivi regitrite arv HIGH (käsud RD (3) ja write multi (10)
modb_r16:;		movf	bytecnt,W				; mitmes bait oli
		;		sublw	.6
		;		btfss	ZERO
		;		goto	modb_r17				; ei
		;		movf	Char,W
;				movwf	mbnregL					; seivi regitrite arv LOW (käsud RD (3) ja write multi (10)
modb_r17:		movf	bytecnt,W
				subwf	countL,W				; saadetis käes (countL-s oodatav baitide arv)?
				btfss	ZERO
				goto	RREnd1					; eip !
;				movf	mbCmnd,W				; kas oli käsk 0x10 (kirjuta mitu reg. korraga)?
				bsf		pank
				movf	Puhver+.1,W
				bcf		pank
				sublw	modbus_wrmulti
				btfss	ZERO
				goto	modb_r2					; ei, pakett käes, kontrolli summat
				btfsc	cmd10
				goto	modb_r2
				bsf		pank
				movf	Puhver+.6,W				; jah, loeme saadetavate baitide arvu ja ootame nende saabumist
				bcf		pank
				addlw	.2
				addwf	countL,F
				bsf		cmd10
				goto	RREnd1					; jääb kuuldele...

modb_r2:		movlw	0xFF					; pakett käes, kontrollime
				movwf	_RS485chkH				; valmistume kontrollsumma arvutamiseks	
				movwf	_RS485chk
				decf	countL,F
				decf	countL,W
				movwf	bytecnt
				movlw	Puhver					; kontrollime summat
				movwf	FSR
modb_r3:		movf	INDF,W
				call	mb_crc16				; kontrollsummeeri
				incf	FSR,F
				decfsz	 bytecnt
				goto	modb_r3
				movf	_RS485chk,W				; kontroll
				subwf	INDF,W
				btfss	ZERO
				goto	reset_ser				; viga
				incf	FSR,F
				movf	_RS485chkH,W			; kontroll
				subwf	INDF,W
				btfss	ZERO
				goto	reset_ser				; viga, eksiteerib via reset_ser
				call	reset_ser				; pakett ok, nulli ikkagi seriali side
				bsf		cmd_ok					; aga märgi ära, et pakett oli ok
				bsf		pank
				bcf		PIE1,RCIE				; enne uut käsku vastuvõtu ei võta kuni senine täidetud
				bcf		pank
				return
;===============================================
; ********* käskude täitmine *******************
;===============================================
command:		movlw	Puhver					; pakett OK, täidame käsu !
				movwf	FSR
				bsf		pank1
				movf	Register274+.1,W		; 1. bait on slave aadress. Kas jutt mulle ?
				bcf		pank1
				subwf	INDF,W
				btfsc	ZERO
				goto	rcv_1					; jutt minule
				movf	INDF,W					; kas broadcast (adr. 0x00) ?
				addlw	.0
				btfss	ZERO
				goto	reset_ser1				; viga
;----------- CHG side reload iga baidi kuulmisel ------------------
rcv_1:
;				pagesel	reload_side
;				call	reload_side				; sidetaimeri reload
;				pagesel	rcv_1
;----------- CHG side reload iga baidi kuulmisel ------------------
				incf	FSR,F
				movf	INDF,W					; kas oli käsk RD holding register (0x03) ?
				sublw	modbus_rd
				btfsc	ZERO
				goto	modb_read				; jah
				movf	INDF,W					; kas oli käsk WR holding register (0x06) ?
				sublw	modbus_wr
				btfsc	ZERO
				goto	modb_write				; jah
				movf	INDF,W					; kas oli käsk WR conf register (0x45) ?
				sublw	modbus_cnf
				btfsc	ZERO
				goto	modb_conf				; jah
				goto	valekask				; teavita et oli vale käsk
;;===============================================
;; sidetaimeri reload
;;===============================================
;reload_side:	bsf		sidetmron
;				movlw	sidetime
;				movwf	sidetaimer
;				bsf		pank1
;				movf	Register277,W			; taasta pulsi kestus
;				bcf		pank1
;				movwf	reset1pulsetmr		
;
;				bsf		pank1
;				movf	Register277+.1,W		; taasta side kadumise viiteaeg reseti 1 generaatorile
;				bcf		pank1
;				movwf	reset1dlytmr
;				bsf		reset1					; reseti 1 pinn maha (igaks juhux)
;				bcf		n_reset1				; inversioon
;
;				bcf		reset1pulseon			; reseti 1 pulsi generaator OHV (igaks juhux)
;rls1:			bsf		pank1
;				movf	Register279+.0,W		; taasta pulsi kestus
;				bcf		pank1
;				movwf	reset2pulsetmr			
;
;				bsf		pank1
;				movf	Register279+.1,W		; side kadumise viiteaeg reseti 2 generaatorile
;				bcf		pank1
;				movwf	reset2dlytmr
;				bcf		PWRSW					; reseti 2 pinn maha (igaks juhux)
;				bcf		reset2pulseon			; reseti 2 pulsi generaator OHV (igaks juhux)
;rls2:			return
;===============================================
; loe N registrit
;===============================================
modb_read:		call	ch_algadr				; kas loetava algusaadress on piirides ?
				btfsc	CARRY
				goto	valedata				; ei ole, nii ütlegi
mbr0:			call	chk_len					; ja lõpp ?
				btfsc	CARRY
				goto	valedata				; per...
				btfss	clrsticky				; lugesime reg.1 ? (suvalises kombinatsioonis teistega)
				goto	mbr1
				movlw	0xFF					
				movf	dinpress
				clrf	dinsticky				; jah, siis nullime vastava sticky-baidi sest master sai teavitet
				clrf	ainsticky				; jah, siis nullime vastava sticky-baidi sest master sai teavitet
				bcf		clrsticky				
mbr1:			bcf		writeit					; oli lugemine, ei ole vaja kirjutada
				bsf		pank
				rlf		m_radrL,W				; liidame puhvri alguse (aadress *2)
				bcf		pank
				addlw	Register0
				movwf	adrtemp					; loetava registri aadress olemas !
				bsf		pank
				movf	n_regH,W				; loetavate registrite arv (HIGH)
				bcf		pank
				movwf	countH
				bsf		pank
				rlf		n_regL,W				; loetavate registrite arv (LOW)
				bcf		pank
				movwf	countL
; Vastus: ADR, CMND, NoB, DataH,DataL,..., CRCH, CRCL				
;				movf	mbCmnd,W				; vastame: käsukood puhvrisse
;				bsf		pank
;				movwf	Puhver+.1
;				bcf		pank
				call	paketialgus				; kirjuta paketi algus (ADR, CMND) puhvrisse	
				movf	countL,W				; NOB
				movwf	INDF
				incf	FSR,F
				call	mb_crc16
				incf	bytecnt,F				; loendame baite
				movf	adrtemp,W				; daata enese aadress
				btfsc	dallas
				addlw	.34
				movwf	source
				movlw	.3						; kirjutame puhvrisse alates pos. 3 (enne ADR,CMD,NOB)
				movwf	destination
modb_read_loop: movf	source,W
				movwf	FSR
				bsf		STATUS,IRP				; asjad on pangas 3
				movf	INDF,W
				bcf		STATUS,IRP		
				movwf	datatemp
				movlw	Puhver
				movwf	FSR
				movf	destination,W
				addwf	FSR,F
				movf	datatemp,W
				movwf	INDF
				incf	destination,F
				call	mb_crc16
				incf	bytecnt,F				; loendame baite
				incf	source,F
				decfsz	countL
				goto	modb_read_loop
				btfss	reg10					; kui loeti registrit 10, võib nüüd wiegandi bitiloendid nullida
				goto	modb_read_end
				btfss	wiegandAto				; wiegand A tulemus käes ?
				goto	modb_r_1				; eip
				clrf	WAbitcount				; jah - nulli A-wiegandi bitiloendid
				bcf		wiegandAto				; nüüd lubame uuesti A-wiegandi lugemise
				bsf		pank1
				clrf	Register10+.0

				clrf	Register11+.0
				clrf	Register11+.1
				clrf	Register12+.0
				clrf	Register12+.1
				clrf	Register13+.0
				clrf	Register13+.1
				clrf	Register14+.0
				clrf	Register14+.1
				bcf		pank1

modb_r_1:		btfss	wiegandBto				; wiegand B tulemus käes ?
				goto	modb_read_end			; eip
				clrf	WBbitcount				; jah - nulli B-wiegandi bitiloendid
				bcf		wiegandBto				; nüüd lubame uuesti B-wiegandi lugemise
				bsf		pank1
				clrf	Register10+.1

				clrf	Register15+.0
				clrf	Register15+.1
				clrf	Register16+.0
				clrf	Register16+.1
				clrf	Register17+.0
				clrf	Register17+.1
				clrf	Register18+.0
				clrf	Register18+.1
				bcf		pank1

modb_read_end:	incf	FSR,F
				goto	paketilopp				; kirjuta CRC puhvrisse, loenda baite ja saada vastus teele
;===============================================
; kirjuta ühte registrit
;===============================================
modb_write:		call	ch_algadr				; kas kirjutatava algusaadress on piirides ja kas lubatud kirjutada ?
				btfsc	CARRY
				goto	valedata				; ei ole, nii ütlegi
				btfsc	readonly				; kirjutamine lubatud ?
				goto	valedata				; ei ole, nii ütlegi
				bsf		pank
				rlf		m_radrL,W				; liidame puhvri alguse
				bcf		pank
				addlw	Register0
				movwf	FSR
				bsf		pank
				movf	n_regH,W				; data (HIGH)
				bcf		pank
				bsf		STATUS,IRP
				movwf	INDF
				bcf		STATUS,IRP
				incf	FSR,F
				bsf		pank
				movf	n_regL,W				; data (LOW)
				bcf		pank
				bsf		STATUS,IRP
				movwf	INDF
				bcf		STATUS,IRP
				btfss	reg0					; oli register 0 ?
				goto	modb_write1				; eip !
				bsf		pank1
				movf	Register0,W				; jah, dubleerime kohe porti ka
				bcf		pank1
				movwf	DoutPort				; digiväljundid ja PU-d
				bsf		pank1
				movf	Register0+.1,W	
				bcf		pank1
				movwf	Registertemp7
;----
				andlw	0x0F
				movwf	PORTA					; ja bitikaupa miskipärast ei lähe !?
				btfsc	Registertemp7,.4		; kombineerime baidid ANA-pordi juhtimisex	
				bsf		PORTA,.5
;				addlw	0x20					; analoog või sisendi puhul kirjutamine nagunii ei mõju
				bcf		CARRY
				rrf		Registertemp7,F
				swapf	Registertemp7,W
				andlw	0x07
				movwf	PORTE

;				movf	Registertemp3,W			; Register0+1 (UIO väljundid) sisus kirjutame sisendid ja analoogpinnid 0-i
;				andwf	Registertemp1,W
;				bsf		pank1
;				andwf	Register0+.1,F
;				bcf		pank1
modb_write1:	bcf		reg0				
; Vastus: ADR, CMND, ADRH,ADRL,DATAH,DATAL, CRCH,CRCL
;				movf	mbCmnd,W				; vastame: käsukood puhvrisse
;				bsf		pank
;				movwf	Puhver+.1
;				bcf		pank
				call	paketialgus				; kirjuta paketi algus (ADR, CMND) puhvrisse	
;				movf	mbRAdrH,W				; kirjutatud reg. aadress HIGH, puhvrist, modimata kujul
				bsf		pank
				movf	Puhver+RSBufLen-.1,W;Puhver+.2,W
				bcf		pank
				movwf	INDF
				incf	FSR,F
				call	mb_crc16
				incf	bytecnt,F				; loendame baite
;				movf	mbRAdrL,W				; kirjutatud reg. aadress LOW, puhvrist, modimata kujul
				bsf		pank
				movf	Puhver+RSBufLen,W;Puhver+.3,W
				bcf		pank
				movwf	INDF
				incf	FSR,F
				call	mb_crc16
				incf	bytecnt,F				; loendame baite
				bsf		pank
				movf	n_regH,W				; data HIGH
				bcf		pank
				movwf	INDF
				incf	FSR,F
				call	mb_crc16
				incf	bytecnt,F				; loendame baite
				bsf		pank
				movf	n_regL,W				; data LOW
				bcf		pank
				movwf	INDF
				incf	FSR,F
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
				incf	FSR,F					; viita seerianumbrile (HIGH)
				bsf		pank1
				movf	Register258+.1,W		; õige ?
				bcf		pank1
				subwf	INDF,W
				btfss	ZERO
				goto	reset_ser				; jama !
				incf	FSR,F					; viita seerianumbrile (LOW)
				bsf		pank1
				movf	Register259+.1,W		; õige ?
				bcf		pank1
				subwf	INDF,W
				btfss	ZERO
				goto	reset_ser				; jama !
				bsf		writeit					; paneme peale vastamist parameetrid kirja ka !
				incf	FSR,F					; üle ser. nr. korduse
				incf	FSR,F					; üle ser. nr. korduse
				incf	FSR,F					; üle ser. nr. korduse
				movf	INDF,W					; pull-up mask
				bsf		pank1
				movwf	Register272+.1
				clrf	Register272
				bcf		pank1
				incf	FSR,F					
				movf	INDF,W					; seriali parameetrid
;				movwf	sampledly;serpartemp				; ei kirjuta õigesse kohta, veidi hiljem
				bsf		pank1
				movwf	Register273+.1			; kohe RAM'i kirja, save_setup kirjutab EEPROMi
				clrf	Register273
				bcf		pank1
				incf	FSR,F					
				movf	INDF,W					; modbussi aadress
				bsf		pank1
				movwf	Register274+.1
				clrf	Register274
				bcf		pank1
				incf	FSR,F					
				movf	INDF,W					; IO suund
				bsf		pank1
				movwf	Register275+.1
				clrf	Register275
				bcf		pank1
; Vastus: ADR, CMD, ID1, ID2, ID1, ID2, CRCH,CRCL
;				movf	mbCmnd,W				; vastame: käsukood puhvrisse
;				bsf		pank
;				movwf	Puhver+.1
;				bcf		pank
				call	paketialgus				; kirjuta paketi algus (ADR, CMND) puhvrisse	
				bsf		pank1
				movf	Register258+.1,W		; serial nr.  HIGH
				bcf		pank1
				movwf	INDF
				incf	FSR,F
				call	mb_crc16
				incf	bytecnt,F				; loendame baite
				bsf		pank1
				movf	Register259+.1,W		; serial nr.  LOW
				bcf		pank1
				movwf	INDF
				incf	FSR,F
				call	mb_crc16
				incf	bytecnt,F				; loendame baite
				bsf		pank1
				movf	Register258+.1,W		; serial nr.  HIGH
				bcf		pank1
				movwf	INDF
				incf	FSR,F
				call	mb_crc16
				incf	bytecnt,F				; loendame baite
				bsf		pank1
				movf	Register259+.1,W		; serial nr.  LOW
				bcf		pank1
				movwf	INDF
				incf	FSR,F
				call	mb_crc16
				incf	bytecnt,F				; loendame baite
;				call	paketilopp				; kirjuta CRC puhvrisse, loenda baite ja saada vastus teele. Exiteerib paketilõpu kaudu!
;				movf	sampledly,W;serpartemp,W			; rakendame uued seriali parameetrid alles nüüd
;				bsf		pank1
;				movwf	Register273+.1
;				bcf		pank1
;				return
;===============================================================================
; Abifunktsioonid
;===============================================================================
paketilopp:		movf	_RS485chk,W				; ja summa
				movwf	INDF
				incf	FSR,F
				incf	bytecnt,F				; loendame baite
				movf	_RS485chkH,W	
				movwf	INDF
				incf	FSR,F
				incf	bytecnt,F				; loendame baite
				call	dly						; 3,98 ms viidet
				movlw	Puhver					; hakkab saatma
				movwf	FSR
				bsf		Dir
send1:			movf	INDF,W
				call	SendCHAR
				incf	FSR,F
				decfsz	bytecnt
				goto	send1
				bcf		Dir
				btfss	writeit					; oli konfidaata ja see tuleks EEPROMi kirjutada?
				goto	reset_ser;send2
				bcf		INTCON,GIE
				pagesel	Save_Setup
				call	Save_Setup				; jah
				pagesel	send1
				bsf		INTCON,GIE
				pagesel	reset_ser
send2:			bcf		writeit					; kirjutatud !
				pagesel	reset_ser
;------------ CHG ----------------
				goto	reset_ser1				; lõpetame jama ää...
;===============================================================================
paketialgus:	movlw	Puhver					; formeerime vastuse saatepuhvrisse
				movwf	FSR
				movlw	0xFF					
				movwf	_RS485chkH				; valmistume kontrollsumma arvutamiseks	
				movwf	_RS485chk
				clrf	bytecnt
				bsf		pank1
				movf	Register274+.1,W		; oma aadress
				bcf		pank1
				movwf	INDF
				incf	FSR,F
				call	mb_crc16
				incf	bytecnt,F				; loendame baite
				movf	INDF,W					; käsk juba kirjas, arvutame ta CRC sisse ja loendame baite kah
				incf	FSR,F
				call	mb_crc16
				incf	bytecnt,F				; loendame baite
				return
;===============================================================================
dly:			movlw	0xD5
				movwf	TMR0
				bcf		INTCON,T0IF
dly1:			btfss	INTCON,T0IF				; 5,5 mS viidet
				goto	dly1
				bcf		INTCON,T0IF
				bcf		INTCON,T0IE
				return
;===============================================================================
ch_algadr:		bcf		clrsticky				; oletame, et ei loetud reg.1-e.
				bcf		reg0
				bcf		reg10					; ja et ei loetud ka reg. 10-t					
				bcf		dallas
				bsf		pank

				movf	m_radrL,W
				movwf	Puhver+RSBufLen

				movf	m_radrH,W				; kas register 0 ?

				movwf	Puhver+RSBufLen-.1

				bcf		pank
				addlw	.0
				btfss	ZERO
				goto	ch_alg1
				bsf		pank
				movf	m_radrL,W
				bcf		pank
				addlw	.0
				btfss	ZERO
				goto	ch_alg1
				bsf		reg0					; et alustati registrist 0
				bcf		readonly
				goto	chk_algok				; on r/w register 0, kõik OK
ch_alg1:		bsf		pank
				movf	m_radrL,W				; aadressi kontroll jätkub
				bcf		pank
				sublw	0xFF					; kas device type register ?
				btfsc	ZERO
				goto	chk_alg3				; jah !
				bsf		pank
				movf	m_radrH,W
				bcf		pank
				addlw	.0
				btfss	ZERO
				goto	chk_alg3				; aadress > 256, vaata kas konfiregister		
				goto	chk_alg2				; tavaregistrid, kontrolli et adr <= 18	
chk_alg1:		bsf		readonly				; on vaid loetav register
				bsf		pank
				clrf	m_radrH					; kõrgemate adr. puhul vajalik
				bcf		pank
				goto	chk_algok				; OK
chk_alg3:
				movlw	.2
				bsf		pank
				subwf	m_radrH,W
				bcf		pank
				btfsc	ZERO
				goto	chk_alg8				; vist tegu Dallas juraga

chk_alg3c:		movlw	.1						; kas uptime loendi (1F2,3,4,5)
				bsf		pank
				subwf	m_radrH,W
				bcf		pank
				btfss	ZERO
				goto	chk_alg3e				; eip
				movlw	0xF2
				bsf		pank
				subwf	m_radrL,W
				bcf		pank
				btfss	CARRY
				goto	chk_alg3e				; eip
				bsf		pank
				movlw	0x5C
				movwf	m_radrL
				clrf	m_radrH					
				bcf		pank
				bsf		dallas
				goto	chk_alg1				; jah, see on r/0 register !

chk_alg3e:		movlw	.1
				bsf		pank
				subwf	m_radrH,W
				bcf		pank
				btfss	CARRY
				goto	chk_alg3b				; ei vist
;				bsf		pank
;				movf	m_radrH,W
;				bcf		pank
;				sublw	.1						; kas loendid (0x190 ja edasi)?
;				btfss	ZERO
;				goto	chk_alg3b				; ei vist
;				movlw	0x90-.1
;				bsf		pank
;				subwf	m_radrL,W
;				bcf		pank
;				btfss	CARRY
;				goto	chk_alg3b				; ei vist
				bsf		pank
				movf	m_radrL,W
				bcf		pank
				sublw	0x90-.1						
				btfss	CARRY
				goto	chk_alg7;6				; jah, need on r/w registrid

chk_alg3b:		bsf		pank
				movf	m_radrH,W
				bcf		pank
				sublw	.1						; kas muidu liiga kõrge aadress ?
				btfss	CARRY
				goto	chk_algbad				; jah, per...
				bsf		pank
				movf	m_radrL,W				; reg. >= 256, kas 0x0100, 0x0102 või 0x0103 ?
				bcf		pank
				sublw	0x00;FF
				btfss	ZERO
				goto	chk_alg3d
				movlw	.19						; 256 -> 19, r-only, dev. type
				bsf		pank
				movwf	m_radrL
				bcf		pank
				goto	chk_alg1
;-
chk_alg3d:		bsf		pank
				movf	m_radrL,W				
				bcf		pank
				sublw	0x01
				btfss	ZERO
				goto	chk_alg3a
				movlw	.20						; 257 -> 20, r-only, firmware nr.
				bsf		pank
				movwf	m_radrL
				bcf		pank
				goto	chk_alg1

chk_alg3a:		bsf		pank
				movf	m_radrL,W				
				bcf		pank
				sublw	0x02
				btfss	ZERO
				goto	chk_alg4
				movlw	.21						; 258 -> 21, r-only, serial HGIH
				bsf		pank
				movwf	m_radrL
				bcf		pank
				goto	chk_alg1
chk_alg4:		bsf		pank
				movf	m_radrL,W				
				bcf		pank
				sublw	0x03
				btfss	ZERO
				goto	chk_alg5
				movlw	.22						; 259 -> 22, r-only, serial LOW
				bsf		pank
				movwf	m_radrL
				bcf		pank
				goto	chk_alg1
chk_alg5:		bsf		pank
				movf	m_radrL,W				; adr 261...271 välja !
				bcf		pank
				sublw	LOW(.271-.1)			; kas < 271 ?
				btfsc	CARRY
				goto	chk_algbad				; siis per...
				bsf		pank
				movf	m_radrL,W				; kas > 279;7 ?
				bcf		pank
;				sublw	LOW(.277-.1)
				sublw	LOW(.280-.1)
				btfss	CARRY
				goto	chk_algbad				; siis per...
				bsf		writeit					; oli konfidaata.kirjutada EEPROMi !
				bsf		pank
;				decf	m_radrL,F				; adr 271..279, teisendame (high bait teha nulliks)
;				decf	m_radrL,F
				movlw	.8;7
				addwf	m_radrL,F				; aadress +8;7, high = 0
				clrf	m_radrH					
				bcf		pank
chk_alg6:		bcf		readonly				; on r/w register
				goto	chk_algok
chk_alg2:		bsf		pank
				movf	m_radrL,W				; tavaregistrid, kas adr <18 ?
				bcf		pank
				sublw	workregisters-.1
				btfss	CARRY
				goto	chk_algbad				; eip, per...
				bsf		clrsticky				; ja järelikult võib sticky biti ka maha võtta peale lugemist
				bsf		pank
				movf	m_radrL,W
				bcf		pank
				sublw	.10						; kas oli reg. 10 lugemine ?
				btfsc	ZERO
				bsf		reg10
				goto	chk_alg1				; jah, need kõik read-only !
chk_alg7:		bsf		pank					; teisenda loendi aadress: high = 0, low -113
				movlw	.112;3
				subwf	m_radrL,F
				clrf	m_radrH					
				bcf		pank
				goto	chk_alg6				; jah, need on r/w registrid
chk_alg8:		bsf		pank					; kas temp. näidud ( <0x28A) ?
				movf	m_radrL,W
				bcf		pank
				sublw	0x8A-.1						
				btfss	CARRY
				goto	chk_alg9				; eip, tegeletakse andurite ID-dega. Need on r/o ! 
				bsf		pank					; teisenda anduri aadress: high = 0, low -41
				movlw	.41;(.41-.34)
				subwf	m_radrL,F
				clrf	m_radrH					
				bcf		pank
				bsf		dallas
				goto	chk_alg1				; jah, need on r/0 registrid !
				
chk_alg9:		bsf		pank
				movf	m_radrL,W
				bcf		pank
				sublw 	LOW(.767)				; teha reset (äksessiti reg. 767) ?
				btfsc	ZERO
				goto	chk_alg10				; jah
				bsf		pank					; teisenda anduri ID aadress: high = 0, low -73
				movlw	.82;(.73-.34)
				subwf	m_radrL,F
				clrf	m_radrH					
				bcf		pank
				bsf		dallas
				goto	chk_alg1				; jah, need on r/0 registrid !
chk_alg10:	;	bsf		dal_discovery
	pagesel saabas
	goto	saabas
				goto	0x0000

chk_algok:		bcf		CARRY					; algusaadress oli lubatud piirides => Cy=0
				return
chk_algbad:		bsf		CARRY					; algusaadress oli üle piiri => Cy=1
				bcf		clrsticky	
				return
;===============================================================================
chk_len:		bsf		pank
				movf	n_regH,W				; loetavate registrite arv (HIGH)
				bcf		pank
				addlw	.0
				btfss	ZERO
				goto	chk_lenbad				; liiga pikk lugemise/kirjutamise soov -> per...
				bsf		pank
				movf	n_regL,W				; loetavate registrite arv (LOW)
				addwf	m_radrL,W				; mitut registrit sooviti töödelda ?
				bcf		pank
				sublw	maxregisters
				btfss	CARRY
				goto	chk_lenbad
				bsf		clrsticky				; oletame, et lubataxe sticky't maha võtta

				btfss	reg0					; kas vaid 1 register & reg0 ? siis clrsticky =0 !!!
				goto	chk_len1				; ei, siis kõik kenasti
				bsf		pank					; jah, vaatame kas pikkus oli 1
				movf	n_regL,W				; loetavate registrite arv (LOW)
				bcf		pank
				sublw	.1
;				btfsc	ZERO
				btfss	ZERO
				goto	chk_len1				; ei, siis kõik kenasti
				bcf		clrsticky				; jah, siis oli vaid reg. 0 lugemine => ei luba sticky't maha võtta
chk_len1:		bcf		CARRY					; lõppaadress oli lubatud piirides => Cy=0
				return
chk_lenbad:		bsf		CARRY					; lõppaadress oli üle piiri => Cy=1
				bcf		clrsticky	
				return
;===============================================================================
valekask:		movlw	IllFunc					; vale käsk, mine per...
				goto	send_err
valedata:		movlw	IllDAdr
send_err:		bsf		pank
				movwf	Puhver+.2				; vea kood puhvrisse
				bcf		pank
				movlw	Puhver					; formeerime vastuse saatepuhvrisse
				movwf	FSR
				movlw	0xFF					
				movwf	_RS485chkH				; valmistume kontrollsumma arvutamiseks	
				movwf	_RS485chk
				clrf	bytecnt
				bsf		pank1
				movf	Register274+.1,W		; oma aadress
				bcf		pank1
				movwf	INDF
				incf	FSR,F
				call	mb_crc16
				incf	bytecnt,F				; loendame baite
;---------
;				bsf		mbCmnd,.7				; käsukoodi b7=1 -> vea märk
;				movf	mbCmnd,W
;				movwf	INDF
;				incf	FSR,F
				bsf		pank
				bsf		Puhver+.1,.7
				movf	Puhver+.1,W
				bcf		pank
				incf	FSR,F
;---------
				call	mb_crc16
				incf	bytecnt,F				; loendame baite
				movf	INDF,W					; vea kood siin juba kirjas, arvutame CRC-sse ja loendame ikkagi
				incf	FSR,F
				call	mb_crc16
				incf	bytecnt,F				; loendame baite
				goto	paketilopp				; summa puhvisse ja saada teele
;===============================================================================
reset_ser1:	
				pagesel	reload_side
				call	reload_side				; sidetaimeri reload
				pagesel	reset_ser
reset_ser:		clrf	bytecnt					; - baitide loendi
				bcf		SerialTimeOut			
				bcf		SerialTimerOn			; taimer seisma
				movlw	serialtime
				movwf	serialtimer
				banksel	RCSTA
				bcf		RCSTA,CREN
				bsf		RCSTA,CREN
				banksel	.0
				movf	RCREG,W
				movlw	.8						; ootame 8 baidist paketti
				movwf	countL
				bcf		cmd10
				bcf		cmd_ok
				return
RREnd1:			movlw	serialtime				; relae ootetaimer
				movwf	serialtimer
;----------- CHG side reload iga baidi kuulmisel ------------------
				pagesel	reload_side
				call	reload_side				; sidetaimeri reload
				pagesel	RREnd1
;----------- CHG side reload iga baidi kuulmisel ------------------
				return
;===============================================================================
; ********************** funktsioonid ******************************************
;===============================================================================
inc_count:		bsf		STATUS,IRP				; loendid on pangas 3 !
				bcf		CARRY
				movf	INDF,W
				addlw	.1
				movwf	INDF					; loendame 1 hoolega ära debouncetud pulsi juurde
				btfss	CARRY
				goto	inc_end
				decf	FSR,F
				bcf		CARRY
				movf	INDF,W
				addlw	.1
				movwf	INDF				
				btfss	CARRY
				goto	inc_end
				decf	FSR,F
				bcf		CARRY
				movf	INDF,W
				addlw	.1
				movwf	INDF				
				btfss	CARRY
				goto	inc_end
				decf	FSR,F
				bcf		CARRY
				movf	INDF,W
				addlw	.1
				movwf	INDF				
inc_end:		bcf		STATUS,IRP				; tagasi maa peale...
				return
;===============================================================================
;*********************** Taimer 1'e INT ****************************************
;===============================================================================
; * T1 on süsteemi taimer intervalliga 10 ms.
T1int:			bcf     PIR1,TMR1IF    			; katkestuse nõue maha 
				movlw	T1resoL					; lae taimer 1 uuesti
				movwf	TMR1L
				movlw	T1resoH
				movwf	TMR1H
				clrwdt
;**** Wiegandi paketi lõpu taimerid ****
T1intwiegA:		btfss	wiegandAtmron
				goto	T1intwiegB
				decfsz	wiegandAtimer
				goto	T1intwiegB
				movlw	wiegandtime
				movwf	wiegandAtimer
				bsf		wiegandAto
				bcf		wiegandAtmron
				movf	WAbitcount,W
				bsf		pank1
				movwf	Register10+.0
				bcf		pank1
				clrf	WAbitcount
				btfsc	wiegandBtmron			; kui teise lugeja lugemine pooleli, ei lase veel sidet üles võtta
				goto	T1intwiegB
				bsf		pank
				bsf		PIE1,RCIE
				bcf		pank
T1intwiegB:		btfss	wiegandBtmron
				goto	T1intnw
				decfsz	wiegandBtimer
				goto	T1intnw
				movlw	wiegandtime
				movwf	wiegandBtimer
				bsf		wiegandBto
				bcf		wiegandBtmron
				movf	WBbitcount,W
				bsf		pank1
				movwf	Register10+.1
				bcf		pank1
				clrf	WBbitcount
				btfsc	wiegandAtmron			; kui teise lugeja lugemine pooleli, ei lase veel sidet üles võtta
				goto	T1intnw
				bsf		pank
				bsf		PIE1,RCIE
				bcf		pank
;**** sekundi aja taimerid ****
T1intnw:		decfsz	sekunditmr
				goto	T1int_1
				movlw	sekundiaeg
				movwf	sekunditmr
				pagesel	GetTemp
				call	GetTemp
			pagesel	T1intnw
;**** uptime loendamine ****
T1_uptime:		movlw	LoendiUP+.3
				movwf	FSR
				call	inc_count				; tixub 1 pulsi 
;**** uptime loendamine ****		
				btfss	sidetmron				; kui 30s jooksul meiega ei suheldud, võtab default sideparameetrid
				goto	T1int_0
				decfsz	sidetaimer
				goto	T1int_0
				pagesel	setup_serial0
				call	setup_serial0			; võtab vaikimisi seriali seaded
				pagesel	reset_ser1
				call	reset_ser1				; sidetaimeri reload
				bcf		sidetmron				; sidetaimer seisma
;**** Resettide taimerid ****
T1int_0:
				pagesel	decrtmrs
				call	decrtmrs
				pagesel	T1int_1

;**** sidepaketi taimer ****
T1int_1:		btfss	SerialTimerOn			; seriali taimer käib?
				goto	T1int_2
				decf	serialtimer,F			; aeg täis?
				movf	serialtimer,W
				addlw	.0
				btfss	ZERO
				goto	T1int_2
				call	reset_ser
				bcf		PIR1,RCIF				; katkestuse nõue maha
; *** loendite sisendite debounce ***
T1int_2:		btfss	sens1tmron				; loendi 1: debounceme ?
				goto	T1int_3					; eip
				decfsz	Loendi1tmr
				goto	T1int_3
				movlw	senstime				; on aeg !
				movwf	Loendi1tmr
				bcf		sens1tmron
				movlw	Loendi1+.3
				movwf	FSR
				call	inc_count				; tixub 1 pulsi 
T1int_3:		btfss	sens2tmron				; loendi 2
				goto	T1int_4					
				decfsz	Loendi2tmr
				goto	T1int_4
				movlw	senstime			
				movwf	Loendi2tmr
				bcf		sens2tmron
				movlw	Loendi2+.3
				movwf	FSR
				call	inc_count			
T1int_4:		btfss	sens3tmron				; loendi 3
				goto	T1int_5				
				decfsz	Loendi3tmr
				goto	T1int_5
				movlw	senstime				
				movwf	Loendi3tmr
				bcf		sens3tmron
				movlw	Loendi3+.3
				movwf	FSR
				call	inc_count				
T1int_5:		btfss	sens4tmron				; loendi 4
				goto	T1int_6				
				decfsz	Loendi4tmr
				goto	T1int_6
				movlw	senstime				
				movwf	Loendi4tmr
				bcf		sens4tmron
				movlw	Loendi4+.3
				movwf	FSR
				call	inc_count				
T1int_6:		btfss	sens5tmron				; loendi 5
				goto	T1int_7				
				decfsz	Loendi5tmr
				goto	T1int_7
				movlw	senstime				
				movwf	Loendi5tmr
				bcf		sens5tmron
				movlw	Loendi5+.3
				movwf	FSR
				call	inc_count				
T1int_7:		btfss	sens6tmron				; loendi 6
				goto	T1int_8				
				decfsz	Loendi6tmr
				goto	T1int_8
				movlw	senstime				
				movwf	Loendi6tmr
				bcf		sens6tmron
				movlw	Loendi6+.3
				movwf	FSR
				call	inc_count				
T1int_8:		btfss	sens7tmron				; loendi 7
				goto	T1int_9				
				decfsz	Loendi7tmr
				goto	T1int_9
				movlw	senstime				
				movwf	Loendi7tmr
				bcf		sens7tmron
				movlw	Loendi7+.3
				movwf	FSR
				call	inc_count				
T1int_9:		btfss	sens8tmron				; loendi 8
				goto	Din_0				
				decfsz	Loendi8tmr
				goto	Din_0
				movlw	senstime				
				movwf	Loendi8tmr
				bcf		sens8tmron
				movlw	Loendi8+.3
				movwf	FSR
				call	inc_count				
;**** sisendite debounce ***** - > sisend 0 (DIN plokist)
Din_0:	;		bsf		pank1
		;		movf	Register273+.1,W		; teeb in_sticky bitti sisaldavast reg-st koopia alamällu
		;		bcf		pank1
		;		movwf	Registertemp3
				btfss	in_sticky
				goto	Din_0a
				btfsc	dinstuck0
				goto	Din_1
Din_0a:			btfsc	Din0					; sisend 0 madal ?
				goto	Din0high				; ei, kõrge, seedi seda
				btfss	in_debounce				; kas debounce't teeb ?
				goto	Din_0stk				; ei, arvestab kohe
				btfss	d0timeron				; jah, kas juba teame ?
				goto	Din0high1				; ei, käivita deb. timer kui vaja
				decfsz	din0tmr					; debouncetud?
				goto	Din_1					; ei veel, võta järgmine sisend
Din_0stk:		bsf		dinpress0				; jah, võtame arvesse
				bsf		dinstuck0				; blokeerime biti igal juhul
Din_0setlow:	bsf		pank1
				bsf		Register1,.0
				bcf		pank1
Din0_lstckl:	bcf		d0timeron
				goto	Din_1					; võta järgmine sisend
Din0high:		btfss	dinpress0				; oli kõrge enne ?
				goto	Din_1					; ei, võta järgmine sisend
				btfss	in_debounce				; kas debounce't teeb ?
				goto	Din_0stkh				; ei, arvestab kohe
				btfss	d0timeron				; lahti laskmise debounce käib?
				goto	Din0strt				; ei, paneme käima
				decfsz	din0tmr
				goto	Din_1					; võta järgmine sisend
				bcf		dinpress0				; loeme lahti lastuks
Din_0stkh:		bsf		dinstuck0				; blokeerime biti igal juhul
Din_0sethi:		bsf		pank1
				bcf		Register1,.0
				bcf		pank1
Din0_lstckh:	bcf		d0timeron
				goto	Din_1					; võta järgmine sisend
Din0high1:		btfsc	dinpress0
				goto	Din_1					; võta järgmine sisend
Din0strt:		movlw	debtime					; käivitame debounce taimeri
				movwf	din0tmr
				bsf		d0timeron
;**** sisendite debounce ***** - > sisend 1 (DIN plokist)
Din_1:			btfss	in_sticky
				goto	Din_1a
				btfsc	dinstuck1
				goto	Din_2
Din_1a:			btfsc	Din1					; sisend 1 madal ?
				goto	Din1high				; ei, kõrge, seedi seda
				btfss	in_debounce				; kas debounce't teeb ?
				goto	Din_1stk				; ei, arvestab kohe
				btfss	d1timeron				; jah, kas juba teame ?
				goto	Din1high1				; ei, käivita deb. timer kui vaja
				decfsz	din1tmr					; debouncetud?
				goto	Din_2					; ei veel, võta järgmine sisend
Din_1stk:		bsf		dinpress1				; jah, võtame arvesse
				bsf		dinstuck1				; blokeerime biti igal juhul
Din_1setlow:	bsf		pank1
				bsf		Register1,.1
				bcf		pank1
Din1_lstckl:	bcf		d1timeron
				goto	Din_2					; võta järgmine sisend
Din1high:		btfss	dinpress1				; oli kõrge enne ?
				goto	Din_2					; ei, võta järgmine sisend
				btfss	in_debounce				; kas debounce't teeb ?
				goto	Din_1stkh				; ei, arvestab kohe
				btfss	d1timeron				; lahti laskmise debounce käib?
				goto	Din1strt				; ei, paneme käima
				decfsz	din1tmr
				goto	Din_2					; võta järgmine sisend
				bcf		dinpress1				; loeme lahti lastuks
Din_1stkh:		bsf		dinstuck1				; blokeerime biti igal juhul
Din_1sethi:		bsf		pank1
				bcf		Register1,.1
				bcf		pank1
Din1_lstckh:	bcf		d1timeron
				goto	Din_2					; võta järgmine sisend
Din1high1:		btfsc	dinpress1
				goto	Din_2					; võta järgmine sisend
Din1strt:		movlw	debtime					; käivitame debounce taimeri
				movwf	din1tmr
				bsf		d1timeron
;**** sisendite debounce ***** - > sisend 2 (DIN plokist)
Din_2:			btfss	in_sticky
				goto	Din_2a
				btfsc	dinstuck2
				goto	Din_3
Din_2a:			btfsc	Din2					; sisend 2 madal ?
				goto	Din2high				; ei, kõrge, seedi seda
				btfss	in_debounce				; kas debounce't teeb ?
				goto	Din_2stk				; ei, arvestab kohe
				btfss	d2timeron				; jah, kas juba teame ?
				goto	Din2high1				; ei, käivita deb. timer kui vaja
				decfsz	din2tmr					; debouncetud?
				goto	Din_3					; ei veel, võta järgmine sisend
Din_2stk:		bsf		dinpress2				; jah, võtame arvesse
				bsf		dinstuck2				; blokeerime biti igal juhul
Din_2setlow:	bsf		pank1
				bsf		Register1,.2
				bcf		pank1
Din2_lstckl:	bcf		d2timeron
				goto	Din_3					; võta järgmine sisend
Din2high:		btfss	dinpress2				; oli kõrge enne ?
				goto	Din_3					; ei, võta järgmine sisend
				btfss	in_debounce				; kas debounce't teeb ?
				goto	Din_2stkh				; ei, arvestab kohe
				btfss	d2timeron				; lahti laskmise debounce käib?
				goto	Din2strt				; ei, paneme käima
				decfsz	din2tmr
				goto	Din_3					; võta järgmine sisend
				bcf		dinpress2				; loeme lahti lastuks
Din_2stkh:		bsf		dinstuck2				; blokeerime biti igal juhul
Din_2sethi:		bsf		pank1
				bcf		Register1,.2
				bcf		pank1
Din2_lstckh:	bcf		d2timeron
				goto	Din_3					; võta järgmine sisend
Din2high1:		btfsc	dinpress2
				goto	Din_3					; võta järgmine sisend
Din2strt:		movlw	debtime					; käivitame debounce taimeri
				movwf	din2tmr
				bsf		d2timeron
;**** sisendite debounce ***** - > sisend 3 (DIN plokist)
Din_3:			btfss	in_sticky
				goto	Din_3a
				btfsc	dinstuck3
				goto	Din_4
Din_3a:			btfsc	Din3					; sisend 3 madal ?
				goto	Din3high				; ei, kõrge, seedi seda
				btfss	in_debounce				; kas debounce't teeb ?
				goto	Din_3stk				; ei, arvestab kohe
				btfss	d3timeron				; jah, kas juba teame ?
				goto	Din3high1				; ei, käivita deb. timer kui vaja
				decfsz	din3tmr					; debouncetud?
				goto	Din_4					; ei veel, võta järgmine sisend
Din_3stk:		bsf		dinpress3				; jah, võtame arvesse
;				btfss	in_sticky				; sticky bitti arvestame ?
;				goto	Din_3setlow				; eip
;				btfsc	dinstuck3				; kas muutus veel teavitamata?
;				goto	Din3_lstckl				; teavitus saatmata, bitti ei muuda
				bsf		dinstuck3				; blokeerime biti igal juhul
Din_3setlow:	bsf		pank1
				bsf		Register1,.3
				bcf		pank1
Din3_lstckl:	bcf		d3timeron
				goto	Din_4					; võta järgmine sisend
Din3high:		btfss	dinpress3				; oli kõrge enne ?
				goto	Din_4					; ei, võta järgmine sisend
				btfss	in_debounce				; kas debounce't teeb ?
				goto	Din_3stkh				; ei, arvestab kohe
				btfss	d3timeron				; lahti laskmise debounce käib?
				goto	Din3strt				; ei, paneme käima
				decfsz	din3tmr
				goto	Din_4					; võta järgmine sisend
				bcf		dinpress3				; loeme lahti lastuks
Din_3stkh:;		btfss	in_sticky				; sticky bitti arvestame ?
;				goto	Din_3sethi				; eip
;				btfsc	dinstuck3				; kas muutus veel teavitamata?
;				goto	Din3_lstckh				; teavitus saatmata, bitti ei muuda
				bsf		dinstuck3				; blokeerime biti igal juhul
Din_3sethi:		bsf		pank1
				bcf		Register1,.3
				bcf		pank1
Din3_lstckh:	bcf		d3timeron
				goto	Din_4					; võta järgmine sisend
Din3high1:		btfsc	dinpress3
				goto	Din_4					; võta järgmine sisend
Din3strt:		movlw	debtime					; käivitame debounce taimeri
				movwf	din3tmr
				bsf		d3timeron
;				goto	Din_4					; võta järgmine sisend
;**** sisendite debounce ***** - > sisend 4 (DIN plokist)
Din_4:			btfss	in_sticky
				goto	Din_4a
				btfsc	dinstuck4
				goto	Din_5
Din_4a:			btfsc	Din4					; sisend 4 madal ?
				goto	Din4high				; ei, kõrge, seedi seda
				btfss	in_debounce				; kas debounce't teeb ?
				goto	Din_4stk				; ei, arvestab kohe
				btfss	d4timeron				; jah, kas juba teame ?
				goto	Din4high1				; ei, käivita deb. timer kui vaja
				decfsz	din4tmr					; debouncetud?
				goto	Din_5					; ei veel, võta järgmine sisend
Din_4stk:		bsf		dinpress4				; jah, võtame arvesse
;				btfss	in_sticky				; sticky bitti arvestame ?
;				goto	Din_4setlow				; eip
;				btfsc	dinstuck4				; kas muutus veel teavitamata?
;				goto	Din4_lstckl				; teavitus saatmata, bitti ei muuda
				bsf		dinstuck4				; blokeerime biti igal juhul
Din_4setlow:	bsf		pank1
				bsf		Register1,.4
				bcf		pank1
Din4_lstckl:	bcf		d4timeron
				goto	Din_5					; võta järgmine sisend
Din4high:		btfss	dinpress4				; oli kõrge enne ?
				goto	Din_5					; ei, võta järgmine sisend
				btfss	in_debounce				; kas debounce't teeb ?
				goto	Din_4stkh				; ei, arvestab kohe
				btfss	d4timeron				; lahti laskmise debounce käib?
				goto	Din4strt				; ei, paneme käima
				decfsz	din4tmr
				goto	Din_5					; võta järgmine sisend
				bcf		dinpress4				; loeme lahti lastuks
Din_4stkh:;		btfss	in_sticky				; sticky bitti arvestame ?
;				goto	Din_4sethi				; eip
;				btfsc	dinstuck4				; kas muutus veel teavitamata?
;				goto	Din4_lstckh				; teavitus saatmata, bitti ei muuda
				bsf		dinstuck4				; blokeerime biti igal juhul
Din_4sethi:		bsf		pank1
				bcf		Register1,.4
				bcf		pank1
Din4_lstckh:	bcf		d4timeron
				goto	Din_5					; võta järgmine sisend
Din4high1:		btfsc	dinpress4
				goto	Din_5					; võta järgmine sisend
Din4strt:		movlw	debtime					; käivitame debounce taimeri
				movwf	din4tmr
				bsf		d4timeron
;				goto	Din_5					; võta järgmine sisend
;**** sisendite debounce ***** - > sisend 5 (DIN plokist)
Din_5:			btfss	in_sticky
				goto	Din_5a
				btfsc	dinstuck5
				goto	Din_6
Din_5a:			btfsc	Din5					; sisend 5 madal ?
				goto	Din5high				; ei, kõrge, seedi seda
				btfss	in_debounce				; kas debounce't teeb ?
				goto	Din_5stk				; ei, arvestab kohe
				btfss	d5timeron				; jah, kas juba teame ?
				goto	Din5high1				; ei, käivita deb. timer kui vaja
				decfsz	din5tmr					; debouncetud?
				goto	Din_6					; ei veel, võta järgmine sisend
Din_5stk:		bsf		dinpress5				; jah, võtame arvesse
;				btfss	in_sticky				; sticky bitti arvestame ?
;				goto	Din_5setlow				; eip
;				btfsc	dinstuck5				; kas muutus veel teavitamata?
;				goto	Din5_lstckl				; teavitus saatmata, bitti ei muuda
				bsf		dinstuck5				; blokeerime biti igal juhul
Din_5setlow:	bsf		pank1
				bsf		Register1,.5
				bcf		pank1
Din5_lstckl:	bcf		d5timeron
				goto	Din_6					; võta järgmine sisend
Din5high:		btfss	dinpress5				; oli kõrge enne ?
				goto	Din_6					; ei, võta järgmine sisend
				btfss	in_debounce				; kas debounce't teeb ?
				goto	Din_5stkh				; ei, arvestab kohe
				btfss	d5timeron				; lahti laskmise debounce käib?
				goto	Din5strt				; ei, paneme käima
				decfsz	din5tmr
				goto	Din_6					; võta järgmine sisend
				bcf		dinpress5				; loeme lahti lastuks
Din_5stkh:;		btfss	in_sticky				; sticky bitti arvestame ?
;				goto	Din_5sethi				; eip
;				btfsc	dinstuck5				; kas muutus veel teavitamata?
;				goto	Din5_lstckh				; teavitus saatmata, bitti ei muuda
				bsf		dinstuck5				; blokeerime biti igal juhul
Din_5sethi:		bsf		pank1
				bcf		Register1,.5
				bcf		pank1
Din5_lstckh:	bcf		d5timeron
				goto	Din_6					; võta järgmine sisend
Din5high1:		btfsc	dinpress5
				goto	Din_6					; võta järgmine sisend
Din5strt:		movlw	debtime					; käivitame debounce taimeri
				movwf	din5tmr
				bsf		d5timeron
;				goto	Din_6					; võta järgmine sisend
;**** sisendite debounce ***** - > sisend 6 (DIN plokist)
Din_6:			btfss	in_sticky
				goto	Din_6a
				btfsc	dinstuck6
				goto	Din_7
Din_6a:			btfsc	Din6					; sisend 6 madal ?
				goto	Din6high				; ei, kõrge, seedi seda
				btfss	in_debounce				; kas debounce't teeb ?
				goto	Din_6stk				; ei, arvestab kohe
				btfss	d6timeron				; jah, kas juba teame ?
				goto	Din6high1				; ei, käivita deb. timer kui vaja
				decfsz	din6tmr					; debouncetud?
				goto	Din_7					; ei veel, võta järgmine sisend
Din_6stk:		bsf		dinpress6				; jah, võtame arvesse
;				btfss	in_sticky				; sticky bitti arvestame ?
;				goto	Din_6setlow				; eip
;				btfsc	dinstuck6				; kas muutus veel teavitamata?
;				goto	Din6_lstckl				; teavitus saatmata, bitti ei muuda
				bsf		dinstuck6				; blokeerime biti igal juhul
Din_6setlow:	bsf		pank1
				bsf		Register1,.6
				bcf		pank1
Din6_lstckl:	bcf		d6timeron
				goto	Din_7					; võta järgmine sisend
Din6high:		btfss	dinpress6				; oli kõrge enne ?
				goto	Din_7					; ei, võta järgmine sisend
				btfss	in_debounce				; kas debounce't teeb ?
				goto	Din_6stkh				; ei, arvestab kohe
				btfss	d6timeron				; lahti laskmise debounce käib?
				goto	Din6strt				; ei, paneme käima
				decfsz	din6tmr
				goto	Din_7					; võta järgmine sisend
				bcf		dinpress6				; loeme lahti lastuks
Din_6stkh:	;	btfss	in_sticky				; sticky bitti arvestame ?
;				goto	Din_6sethi				; eip
;				btfsc	dinstuck6				; kas muutus veel teavitamata?
;				goto	Din6_lstckh				; teavitus saatmata, bitti ei muuda
				bsf		dinstuck6				; blokeerime biti igal juhul
Din_6sethi:		bsf		pank1
				bcf		Register1,.6
				bcf		pank1
Din6_lstckh:	bcf		d6timeron
				goto	Din_7					; võta järgmine sisend
Din6high1:		btfsc	dinpress6
				goto	Din_7					; võta järgmine sisend
Din6strt:		movlw	debtime					; käivitame debounce taimeri
				movwf	din6tmr
				bsf		d6timeron
;				goto	Din_7					; võta järgmine sisend
;**** sisendite debounce ***** - > sisend 7 (DIN plokist)
Din_7:			btfss	in_sticky
				goto	Din_7a
				btfsc	dinstuck7
				goto	Din_8
Din_7a:			btfsc	Din7					; sisend 7 madal ?
				goto	Din7high				; ei, kõrge, seedi seda
				btfss	in_debounce				; kas debounce't teeb ?
				goto	Din_7stk				; ei, arvestab kohe
				btfss	d7timeron				; jah, kas juba teame ?
				goto	Din7high1				; ei, käivita deb. timer kui vaja
				decfsz	din7tmr					; debouncetud?
				goto	Din_8					; ei veel, võta järgmine sisend
Din_7stk:		bsf		dinpress7				; jah, võtame arvesse
;				btfss	in_sticky				; sticky bitti arvestame ?
;				goto	Din_7setlow				; eip
;				btfsc	dinstuck7				; kas muutus veel teavitamata?
;				goto	Din7_lstckl				; teavitus saatmata, bitti ei muuda
				bsf		dinstuck7				; blokeerime biti igal juhul
Din_7setlow:	bsf		pank1
				bsf		Register1,.7
				bcf		pank1
Din7_lstckl:	bcf		d7timeron
				goto	Din_8					; võta järgmine sisend
Din7high:		btfss	dinpress7				; oli kõrge enne ?
				goto	Din_8					; ei, võta järgmine sisend
				btfss	in_debounce				; kas debounce't teeb ?
				goto	Din_7stkh				; ei, arvestab kohe
				btfss	d7timeron				; lahti laskmise debounce käib?
				goto	Din7strt				; ei, paneme käima
				decfsz	din7tmr
				goto	Din_8					; võta järgmine sisend
				bcf		dinpress7				; loeme lahti lastuks
Din_7stkh:;		btfss	in_sticky				; sticky bitti arvestame ?
;				goto	Din_7sethi				; eip
;				btfsc	dinstuck7				; kas muutus veel teavitamata?
;				goto	Din7_lstckh				; teavitus saatmata, bitti ei muuda
				bsf		dinstuck7				; blokeerime biti igal juhul
Din_7sethi:		bsf		pank1
				bcf		Register1,.7
				bcf		pank1
Din7_lstckh:	bcf		d7timeron
				goto	Din_8					; võta järgmine sisend
Din7high1:		btfsc	dinpress7
				goto	Din_8					; võta järgmine sisend
Din7strt:		movlw	debtime					; käivitame debounce taimeri
				movwf	din7tmr
				bsf		d7timeron
;				goto	Din_8					; võta järgmine sisend
Din_8:
;___________________________
;**** sisendite debounce ***** - > sisend 0 (AIN plokist)
Ain_0:	;		bsf		pank1
		;		movf	Register275+.1,W		; teeb registritest koopia alamällu
		;		bcf		pank1
		;		movwf	Registertemp1
		;		bsf		pank1
		;		movf	Register271+.1,W		; teeb registritest koopia alamällu
		;		bcf		pank1
		;		movwf	Registertemp2
				movf	PORTA,W					; loeme ANA-pordi seisu
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
				movf	Registertemp4,W			; lisa: XORime tulemuse sellega läbi
				xorwf	Registertemp5,F;W

				btfsc	Registertemp3,.0		; kas on sisend ?
				goto	Ain_0sethi				; eip, kirjutab nulli registrisse
				btfss	Registertemp1,.0		; kas on digisisend ?
				goto	Ain_0sethi				; eip, kirjutab nulli registrisse
				btfss	in_sticky
				goto	Ain_0a
				btfsc	ainstuck0
				goto	Ain_1
Ain_0a:			btfsc	Ana0					; sisend 0 madal ?
				goto	Ain0high				; ei, kõrge, seedi seda
				btfss	in_debounce				; kas debounce't teeb ?
				goto	Ain_0stk				; ei, arvestab kohe
				btfss	a0timeron				; jah, kas juba teame ?
				goto	Ain0high1				; ei, käivita deb. timer kui vaja
				decfsz	ain0tmr					; debouncetud?
				goto	Ain_1					; ei veel, võta järgmine sisend
Ain_0stk:		bsf		ainpress0				; jah, võtame arvesse
;				btfss	in_sticky				; sticky bitti arvestame ?
;				goto	Ain_0setlow				; eip
;				btfsc	ainstuck0				; kas muutus veel teavitamata?
;				goto	Ain0_lstckl				; teavitus saatmata, bitti ei muuda
				bsf		ainstuck0				; blokeerime biti igal juhul
Ain_0setlow:	bsf		pank1
				bsf		Register1+.1,.0
				bcf		pank1
Ain0_lstckl:	bcf		a0timeron
				goto	Ain_1					; võta järgmine sisend
Ain0high:		btfss	ainpress0				; oli kõrge enne ?
				goto	Ain_1					; ei, võta järgmine sisend
				btfss	in_debounce				; kas debounce't teeb ?
				goto	Ain_0stkh				; ei, arvestab kohe
				btfss	a0timeron				; lahti laskmise debounce käib?
				goto	Ain0strt				; ei, paneme käima
				decfsz	ain0tmr
				goto	Ain_1					; võta järgmine sisend
				bcf		ainpress0				; loeme lahti lastuks
Ain_0stkh:;		btfss	in_sticky				; sticky bitti arvestame ?
;				goto	Ain_0sethi				; eip
;				btfsc	ainstuck0				; kas muutus veel teavitamata?
;				goto	Ain0_lstckh				; teavitus saatmata, bitti ei muuda
				bsf		ainstuck0				; blokeerime biti igal juhul
Ain_0sethi:		bsf		pank1
				bcf		Register1+.1,.0
				bcf		pank1
Ain0_lstckh:	bcf		a0timeron
				goto	Ain_1					; võta järgmine sisend
Ain0high1:		btfsc	ainpress0
				goto	Ain_1					; võta järgmine sisend
Ain0strt:		movlw	debtime					; käivitame debounce taimeri
				movwf	ain0tmr
				bsf		a0timeron
;				goto	Ain_1					; võta järgmine sisend
;**** sisendite debounce ***** - > sisend 1 (AIN plokist)
Ain_1:			btfsc	Registertemp3,.1		; kas on sisend ?
				goto	Ain_1sethi				; eip, kirjutab nulli registrisse
				btfss	Registertemp1,.1		; kas on digisisend ?
				goto	Ain_1sethi				; eip, kirjutab nulli registrisse
				btfss	in_sticky
				goto	Ain_1a
				btfsc	ainstuck1
				goto	Ain_2
Ain_1a:			btfsc	Ana1					; sisend 1 madal ?
				goto	Ain1high				; ei, kõrge, seedi seda
				btfss	in_debounce				; kas debounce't teeb ?
				goto	Ain_1stk				; ei, arvestab kohe
				btfss	a1timeron				; jah, kas juba teame ?
				goto	Ain1high1				; ei, käivita deb. timer kui vaja
				decfsz	ain1tmr					; debouncetud?
				goto	Ain_2					; ei veel, võta järgmine sisend
Ain_1stk:		bsf		ainpress1				; jah, võtame arvesse
;				btfss	in_sticky				; sticky bitti arvestame ?
;				goto	Ain_1setlow				; eip
;				btfsc	ainstuck1				; kas muutus veel teavitamata?
;				goto	Ain1_lstckl				; teavitus saatmata, bitti ei muuda
				bsf		ainstuck1				; blokeerime biti igal juhul
Ain_1setlow:	bsf		pank1
				bsf		Register1+.1,.1
				bcf		pank1
Ain1_lstckl:	bcf		a1timeron
				goto	Ain_2					; võta järgmine sisend
Ain1high:		btfss	ainpress1				; oli kõrge enne ?
				goto	Ain_2					; ei, võta järgmine sisend
				btfss	in_debounce				; kas debounce't teeb ?
				goto	Ain_1stkh				; ei, arvestab kohe
				btfss	a1timeron				; lahti laskmise debounce käib?
				goto	Ain1strt				; ei, paneme käima
				decfsz	ain1tmr
				goto	Ain_2					; võta järgmine sisend
				bcf		ainpress1				; loeme lahti lastuks
Ain_1stkh:;		btfss	in_sticky				; sticky bitti arvestame ?
;				goto	Ain_1sethi				; eip
;				btfsc	ainstuck1				; kas muutus veel teavitamata?
;				goto	Ain1_lstckh				; teavitus saatmata, bitti ei muuda
				bsf		ainstuck1				; blokeerime biti igal juhul
Ain_1sethi:		bsf		pank1
				bcf		Register1+.1,.1
				bcf		pank1
Ain1_lstckh:	bcf		a1timeron
				goto	Ain_2					; võta järgmine sisend
Ain1high1:		btfsc	ainpress1
				goto	Ain_2					; võta järgmine sisend
Ain1strt:		movlw	debtime					; käivitame debounce taimeri
				movwf	ain1tmr
				bsf		a1timeron
;				goto	Ain_2					; võta järgmine sisend
;**** sisendite debounce ***** - > sisend 2 (AIN plokist)
Ain_2:			btfsc	Registertemp3,.2		; kas on sisend ?
				goto	Ain_2sethi				; eip, kirjutab nulli registrisse
				btfss	Registertemp1,.2		; kas on digisisend ?
				goto	Ain_2sethi				; eip, kirjutab nulli registrisse
				btfss	in_sticky
				goto	Ain_2a
				btfsc	ainstuck2
				goto	Ain_3
Ain_2a:			btfsc	Ana2					; sisend 2 madal ?
				goto	Ain2high				; ei, kõrge, seedi seda
				btfss	in_debounce				; kas debounce't teeb ?
				goto	Ain_2stk				; ei, arvestab kohe
				btfss	a2timeron				; jah, kas juba teame ?
				goto	Ain2high1				; ei, käivita deb. timer kui vaja
				decfsz	ain2tmr					; debouncetud?
				goto	Ain_3					; ei veel, võta järgmine sisend
Ain_2stk:		bsf		ainpress2				; jah, võtame arvesse
;				btfss	in_sticky				; sticky bitti arvestame ?
;				goto	Ain_2setlow				; eip
;				btfsc	ainstuck2				; kas muutus veel teavitamata?
;				goto	Ain2_lstckl				; teavitus saatmata, bitti ei muuda
				bsf		ainstuck2				; blokeerime biti igal juhul
Ain_2setlow:	bsf		pank1
				bsf		Register1+.1,.2
				bcf		pank1
Ain2_lstckl:	bcf		a2timeron
				goto	Ain_3					; võta järgmine sisend
Ain2high:		btfss	ainpress2				; oli kõrge enne ?
				goto	Ain_3					; ei, võta järgmine sisend
				btfss	in_debounce				; kas debounce't teeb ?
				goto	Ain_2stkh				; ei, arvestab kohe
				btfss	a2timeron				; lahti laskmise debounce käib?
				goto	Ain2strt				; ei, paneme käima
				decfsz	ain2tmr
				goto	Ain_3					; võta järgmine sisend
				bcf		ainpress2				; loeme lahti lastuks
Ain_2stkh:;		btfss	in_sticky				; sticky bitti arvestame ?
;				goto	Ain_2sethi				; eip
;				btfsc	ainstuck2				; kas muutus veel teavitamata?
;				goto	Ain2_lstckh				; teavitus saatmata, bitti ei muuda
				bsf		ainstuck2				; blokeerime biti igal juhul
Ain_2sethi:		bsf		pank1
				bcf		Register1+.1,.2
				bcf		pank1
Ain2_lstckh:	bcf		a2timeron
				goto	Ain_3					; võta järgmine sisend
Ain2high1:		btfsc	ainpress2
				goto	Ain_3					; võta järgmine sisend
Ain2strt:		movlw	debtime					; käivitame debounce taimeri
				movwf	ain2tmr
				bsf		a2timeron
;				goto	Ain_3					; võta järgmine sisend
;**** sisendite debounce ***** - > sisend 3 (AIN plokist)
Ain_3:			btfsc	Registertemp3,.3		; kas on sisend ?
				goto	Ain_3sethi				; eip, kirjutab nulli registrisse
				btfss	Registertemp1,.3		; kas on digisisend ?
				goto	Ain_3sethi				; eip, kirjutab nulli registrisse
				btfss	in_sticky
				goto	Ain_3a
				btfsc	ainstuck3
				goto	Ain_4
Ain_3a:			btfsc	Ana3					; sisend 3 madal ?
				goto	Ain3high				; ei, kõrge, seedi seda
				btfss	in_debounce				; kas debounce't teeb ?
				goto	Ain_3stk				; ei, arvestab kohe
				btfss	a3timeron				; jah, kas juba teame ?
				goto	Ain3high1				; ei, käivita deb. timer kui vaja
				decfsz	ain3tmr					; debouncetud?
				goto	Ain_4					; ei veel, võta järgmine sisend
Ain_3stk:		bsf		ainpress3				; jah, võtame arvesse
;				btfss	in_sticky				; sticky bitti arvestame ?
;				goto	Ain_3setlow				; eip
;				btfsc	ainstuck3				; kas muutus veel teavitamata?
;				goto	Ain3_lstckl				; teavitus saatmata, bitti ei muuda
				bsf		ainstuck3				; blokeerime biti igal juhul
Ain_3setlow:	bsf		pank1
				bsf		Register1+.1,.3
				bcf		pank1
Ain3_lstckl:	bcf		a3timeron
				goto	Ain_4					; võta järgmine sisend
Ain3high:		btfss	ainpress3				; oli kõrge enne ?
				goto	Ain_4					; ei, võta järgmine sisend
				btfss	in_debounce				; kas debounce't teeb ?
				goto	Ain_3stkh				; ei, arvestab kohe
				btfss	a3timeron				; lahti laskmise debounce käib?
				goto	Ain3strt				; ei, paneme käima
				decfsz	ain3tmr
				goto	Ain_4					; võta järgmine sisend
				bcf		ainpress3				; loeme lahti lastuks
Ain_3stkh:;		btfss	in_sticky				; sticky bitti arvestame ?
;				goto	Ain_3sethi				; eip
;				btfsc	ainstuck3				; kas muutus veel teavitamata?
;				goto	Ain3_lstckh				; teavitus saatmata, bitti ei muuda
				bsf		ainstuck3				; blokeerime biti igal juhul
Ain_3sethi:		bsf		pank1
				bcf		Register1+.1,.3
				bcf		pank1
Ain3_lstckh:	bcf		a3timeron
				goto	Ain_4					; võta järgmine sisend
Ain3high1:		btfsc	ainpress3
				goto	Ain_4					; võta järgmine sisend
Ain3strt:		movlw	debtime					; käivitame debounce taimeri
				movwf	ain3tmr
				bsf		a3timeron
;				goto	Ain_4					; võta järgmine sisend
;**** sisendite debounce ***** - > sisend 4 (AIN plokist)
Ain_4:			btfsc	Registertemp3,.4		; kas on sisend ?
				goto	Ain_4sethi				; eip, kirjutab nulli registrisse
				btfss	Registertemp1,.4		; kas on digisisend ?
				goto	Ain_4sethi				; eip, kirjutab nulli registrisse
				btfss	in_sticky
				goto	Ain_4a
				btfsc	ainstuck4
				goto	Ain_5
Ain_4a:			btfsc	Ana4					; sisend 4 madal ?
				goto	Ain4high				; ei, kõrge, seedi seda
				btfss	in_debounce				; kas debounce't teeb ?
				goto	Ain_4stk				; ei, arvestab kohe
				btfss	a4timeron				; jah, kas juba teame ?
				goto	Ain4high1				; ei, käivita deb. timer kui vaja
				decfsz	ain4tmr					; debouncetud?
				goto	Ain_5					; ei veel, võta järgmine sisend
Ain_4stk:		bsf		ainpress4				; jah, võtame arvesse
;				btfss	in_sticky				; sticky bitti arvestame ?
;				goto	Ain_4setlow				; eip
;				btfsc	ainstuck4				; kas muutus veel teavitamata?
;				goto	Ain4_lstckl				; teavitus saatmata, bitti ei muuda
				bsf		ainstuck4				; blokeerime biti igal juhul
Ain_4setlow:	bsf		pank1
				bsf		Register1+.1,.4
				bcf		pank1
Ain4_lstckl:	bcf		a4timeron
				goto	Ain_5					; võta järgmine sisend
Ain4high:		btfss	ainpress4				; oli kõrge enne ?
				goto	Ain_5					; ei, võta järgmine sisend
				btfss	in_debounce				; kas debounce't teeb ?
				goto	Ain_4stkh				; ei, arvestab kohe
				btfss	a4timeron				; lahti laskmise debounce käib?
				goto	Ain4strt				; ei, paneme käima
				decfsz	ain4tmr
				goto	Ain_5					; võta järgmine sisend
				bcf		ainpress4				; loeme lahti lastuks
Ain_4stkh:;		btfss	in_sticky				; sticky bitti arvestame ?
;				goto	Ain_4sethi				; eip
;				btfsc	ainstuck4				; kas muutus veel teavitamata?
;				goto	Ain4_lstckh				; teavitus saatmata, bitti ei muuda
				bsf		ainstuck4				; blokeerime biti igal juhul
Ain_4sethi:		bsf		pank1
				bcf		Register1+.1,.4
				bcf		pank1
Ain4_lstckh:	bcf		a4timeron
				goto	Ain_5					; võta järgmine sisend
Ain4high1:		btfsc	ainpress4
				goto	Ain_5					; võta järgmine sisend
Ain4strt:		movlw	debtime					; käivitame debounce taimeri
				movwf	ain4tmr
				bsf		a4timeron
;				goto	Ain_5					; võta järgmine sisend
;**** sisendite debounce ***** - > sisend 5 (AIN plokist)
Ain_5:			btfsc	Registertemp3,.5		; kas on sisend ?
				goto	Ain_5sethi				; eip, kirjutab nulli registrisse
				btfss	Registertemp1,.5		; kas on digisisend ?
				goto	Ain_5sethi				; eip, kirjutab nulli registrisse
				btfss	in_sticky
				goto	Ain_5a
				btfsc	ainstuck5
				goto	Ain_6
Ain_5a:			btfsc	Ana5					; sisend 5 madal ?
				goto	Ain5high				; ei, kõrge, seedi seda
				btfss	in_debounce				; kas debounce't teeb ?
				goto	Ain_5stk				; ei, arvestab kohe
				btfss	a5timeron				; jah, kas juba teame ?
				goto	Ain5high1				; ei, käivita deb. timer kui vaja
				decfsz	ain5tmr					; debouncetud?
				goto	Ain_6					; ei veel, võta järgmine sisend
Ain_5stk:		bsf		ainpress5				; jah, võtame arvesse
;				btfss	in_sticky				; sticky bitti arvestame ?
;				goto	Ain_5setlow				; eip
;				btfsc	ainstuck5				; kas muutus veel teavitamata?
;				goto	Ain5_lstckl				; teavitus saatmata, bitti ei muuda
				bsf		ainstuck5				; blokeerime biti igal juhul
Ain_5setlow:	bsf		pank1
				bsf		Register1+.1,.5
				bcf		pank1
Ain5_lstckl:	bcf		a5timeron
				goto	Ain_6					; võta järgmine sisend
Ain5high:		btfss	ainpress5				; oli kõrge enne ?
				goto	Ain_6					; ei, võta järgmine sisend
				btfss	in_debounce				; kas debounce't teeb ?
				goto	Ain_5stkh				; ei, arvestab kohe
				btfss	a5timeron				; lahti laskmise debounce käib?
				goto	Ain5strt				; ei, paneme käima
				decfsz	ain5tmr
				goto	Ain_6					; võta järgmine sisend
				bcf		ainpress5				; loeme lahti lastuks
Ain_5stkh:;		btfss	in_sticky				; sticky bitti arvestame ?
;				goto	Ain_5sethi				; eip
;				btfsc	ainstuck5				; kas muutus veel teavitamata?
;				goto	Ain5_lstckh				; teavitus saatmata, bitti ei muuda
				bsf		ainstuck5				; blokeerime biti igal juhul
Ain_5sethi:		bsf		pank1
				bcf		Register1+.1,.5
				bcf		pank1
Ain5_lstckh:	bcf		a5timeron
				goto	Ain_6					; võta järgmine sisend
Ain5high1:		btfsc	ainpress5
				goto	Ain_6					; võta järgmine sisend
Ain5strt:		movlw	debtime					; käivitame debounce taimeri
				movwf	ain5tmr
				bsf		a5timeron
;				goto	Ain_6					; võta järgmine sisend
;**** sisendite debounce ***** - > sisend 6 (AIN plokist)
Ain_6:			btfsc	Registertemp3,.6		; kas on sisend ?
				goto	Ain_6sethi				; eip, kirjutab nulli registrisse
				btfss	Registertemp1,.6		; kas on digisisend ?
				goto	Ain_6sethi				; eip, kirjutab nulli registrisse
				btfss	in_sticky
				goto	Ain_6a
				btfsc	ainstuck6
				goto	Ain_7
Ain_6a:			btfsc	Ana6					; sisend 6 madal ?
				goto	Ain6high				; ei, kõrge, seedi seda
				btfss	in_debounce				; kas debounce't teeb ?
				goto	Ain_6stk				; ei, arvestab kohe
				btfss	a6timeron				; jah, kas juba teame ?
				goto	Ain6high1				; ei, käivita deb. timer kui vaja
				decfsz	ain6tmr					; debouncetud?
				goto	Ain_7					; ei veel, võta järgmine sisend
Ain_6stk:		bsf		ainpress6				; jah, võtame arvesse
;				btfss	in_sticky				; sticky bitti arvestame ?
;				goto	Ain_6setlow				; eip
;				btfsc	ainstuck6				; kas muutus veel teavitamata?
;				goto	Ain6_lstckl				; teavitus saatmata, bitti ei muuda
				bsf		ainstuck6				; blokeerime biti igal juhul
Ain_6setlow:	bsf		pank1
				bsf		Register1+.1,.6
				bcf		pank1
Ain6_lstckl:	bcf		a6timeron
				goto	Ain_7					; võta järgmine sisend
Ain6high:		btfss	ainpress6				; oli kõrge enne ?
				goto	Ain_7					; ei, võta järgmine sisend
				btfss	in_debounce				; kas debounce't teeb ?
				goto	Ain_6stkh				; ei, arvestab kohe
				btfss	a6timeron				; lahti laskmise debounce käib?
				goto	Ain6strt				; ei, paneme käima
				decfsz	ain6tmr
				goto	Ain_7					; võta järgmine sisend
				bcf		ainpress6				; loeme lahti lastuks
Ain_6stkh:	;	btfss	in_sticky				; sticky bitti arvestame ?
;				goto	Ain_6sethi				; eip
;				btfsc	ainstuck6				; kas muutus veel teavitamata?
;				goto	Ain6_lstckh				; teavitus saatmata, bitti ei muuda
				bsf		ainstuck6				; blokeerime biti igal juhul
Ain_6sethi:		bsf		pank1
				bcf		Register1+.1,.6
				bcf		pank1
Ain6_lstckh:	bcf		a6timeron
				goto	Ain_7					; võta järgmine sisend
Ain6high1:		btfsc	ainpress6
				goto	Ain_7					; võta järgmine sisend
Ain6strt:		movlw	debtime					; käivitame debounce taimeri
				movwf	ain6tmr
				bsf		a6timeron
;				goto	Ain_7					; võta järgmine sisend
;**** sisendite debounce ***** - > sisend 7 (AIN plokist)
Ain_7:			btfsc	Registertemp3,.7		; kas on sisend ?
				goto	Ain_7sethi				; eip, kirjutab nulli registrisse
				btfss	Registertemp1,.7		; kas on digisisend ?
				goto	Ain_7sethi				; eip, kirjutab nulli registrisse
				btfss	in_sticky
				goto	Ain_7a
				btfsc	ainstuck7
				goto	Ain_8
Ain_7a:			btfsc	Ana7					; sisend 7 madal ?
				goto	Ain7high				; ei, kõrge, seedi seda
				btfss	in_debounce				; kas debounce't teeb ?
				goto	Ain_7stk				; ei, arvestab kohe
				btfss	a7timeron				; jah, kas juba teame ?
				goto	Ain7high1				; ei, käivita deb. timer kui vaja
				decfsz	ain7tmr					; debouncetud?
				goto	Ain_8					; ei veel, võta järgmine sisend
Ain_7stk:		bsf		ainpress7				; jah, võtame arvesse
;				btfss	in_sticky				; sticky bitti arvestame ?
;				goto	Ain_7setlow				; eip
;				btfsc	ainstuck7				; kas muutus veel teavitamata?
;				goto	Ain7_lstckl				; teavitus saatmata, bitti ei muuda
				bsf		ainstuck7				; blokeerime biti igal juhul
Ain_7setlow:	bsf		pank1
				bsf		Register1+.1,.7
				bcf		pank1
Ain7_lstckl:	bcf		a7timeron
				goto	Ain_8					; võta järgmine sisend
Ain7high:		btfss	ainpress7				; oli kõrge enne ?
				goto	Ain_8					; ei, võta järgmine sisend
				btfss	in_debounce				; kas debounce't teeb ?
				goto	Ain_7stkh				; ei, arvestab kohe
				btfss	a7timeron				; lahti laskmise debounce käib?
				goto	Ain7strt				; ei, paneme käima
				decfsz	ain7tmr
				goto	Ain_8					; võta järgmine sisend
				bcf		ainpress7				; loeme lahti lastuks
Ain_7stkh:;		btfss	in_sticky				; sticky bitti arvestame ?
;				goto	Ain_7sethi				; eip
;				btfsc	ainstuck7				; kas muutus veel teavitamata?
;				goto	Ain7_lstckh				; teavitus saatmata, bitti ei muuda
				bsf		ainstuck7				; blokeerime biti igal juhul
Ain_7sethi:		bsf		pank1
				bcf		Register1+.1,.7
				bcf		pank1
Ain7_lstckh:	bcf		a7timeron
				goto	Ain_8					; võta järgmine sisend
Ain7high1:		btfsc	ainpress7
				goto	Ain_8					; võta järgmine sisend
Ain7strt:		movlw	debtime					; käivitame debounce taimeri
				movwf	ain7tmr
				bsf		a7timeron
Ain_8:	;		movf	Registertemp4,W			; lisa: XORime tulemuse sellega läbi
T1int_end:
				goto	Int_2
;===============================================================================
; ********************** funktsioonid ******************************************
;===============================================================================
;===============================================================================
; ********************** põhiluup **********************************************
;===============================================================================
main:			
				pagesel	init
				call	init					; prose setup
;;		pagesel	main
				movlw	_10sekundiaeg
				movwf	_10sekunditmr
				clrwdt
				bsf		INTCON,PEIE			
				bsf		INTCON,GIE				; lubame kindral-katckestused
tsykkel:		clrwdt
				btfss	cmd_ok					; oli valiidne käsk ?
				goto	adc000;x					; eip, lase edasi
				clrwdt
				call	command					; jah - töötle ja täida 
				clrwdt
				bcf		cmd_ok					; tehtud !
				bcf		PIR1,RCIF
				bsf		pank
				bsf		PIE1,RCIE				; lubame uue käsu vastuvõttu
				bcf		pank
adcx:			btfss	temper					; aeg temperatuuri mõõta ?
				goto	adc000
				clrwdt
				pagesel	GetTemp
				call	GetT					; jepp
				pagesel	adc000
		clrwdt
		bsf	INTCON,GIE
adc000:			clrwdt
				movlw	Register2				; ADC muundamine - käib kõik kanalid läbi
				movwf	FSR
adcchan0:		btfsc	Registertemp3,.0		; kui see bitt on väljund, kirjutame tulemuseks 0
				goto	adc0					; ongi
				btfsc	Registertemp1,.0		; kui see bitt on digisisend, kirjutame tulemuseks 0
				goto	adc0					; ongi
				goto	adc00					; sobib, muundame
adc0:;			call	wr_zero
				bsf		pank1
				clrf	Register2
				clrf	Register2+.1
				bcf		pank1
				goto	adcchan1				; võta järgmine kanal (sisend)
adc00:;			nop
				banksel	ADCON0
				movlw	chan1					; on anasisend, muundame
				movwf	ADCON0
				call	measure

adcchan1:		btfsc	Registertemp3,.1		
				goto	adc1					
				btfsc	Registertemp1,.1		
				goto	adc1			
				goto	adc11
adc1:;			call	wr_zero
				bsf		pank1
				clrf	Register3
				clrf	Register3+.1
				bcf		pank1
				goto	adcchan2
adc11:;			nop
				banksel	ADCON0
				movlw	chan2
				movwf	ADCON0
				call	measure

adcchan2:		btfsc	Registertemp3,.2		
				goto	adc2					
				btfsc	Registertemp1,.2		
				goto	adc2			
				goto	adc22
adc2:;			call	wr_zero
				bsf		pank1
				clrf	Register4
				clrf	Register4+.1
				bcf		pank1
				goto	adcchan3
adc22:;			nop
				banksel	ADCON0
				movlw	chan3
				movwf	ADCON0
				call	measure

adcchan3:		btfsc	Registertemp3,.3		
				goto	adc3					
				btfsc	Registertemp1,.3		
				goto	adc3			
				goto	adc33
adc3:;			call	wr_zero
				bsf		pank1
				clrf	Register5
				clrf	Register5+.1
				bcf		pank1
				goto	adcchan4
adc33:	;		nop
				banksel	ADCON0
				movlw	chan4
				movwf	ADCON0
				call	measure

adcchan4:		btfsc	Registertemp3,.4		
				goto	adc4					
				btfsc	Registertemp1,.4		
				goto	adc4			
				goto	adc44
adc4:;			call	wr_zero
				bsf		pank1
				clrf	Register6
				clrf	Register6+.1
				bcf		pank1
				goto	adcchan5
adc44:	;		nop
				banksel	ADCON0
				movlw	chan5
				movwf	ADCON0
				call	measure

adcchan5:		btfsc	Registertemp3,.5		
				goto	adc5					
				btfsc	Registertemp1,.5		
				goto	adc5			
				goto	adc55
adc5:;			call	wr_zero
				bsf		pank1
				clrf	Register7
				clrf	Register7+.1
				bcf		pank1
				goto	adcchan6
adc55:;			nop
				banksel	ADCON0
				movlw	chan6
				movwf	ADCON0
				call	measure

adcchan6:		btfsc	Registertemp3,.6		
				goto	adc6					
				btfsc	Registertemp1,.6		
				goto	adc6			
				goto	adc66
adc6:;			call	wr_zero
				bsf		pank1
				clrf	Register8
				clrf	Register8+.1
				bcf		pank1
				goto	adcchan7
adc66:	;		nop
				banksel	ADCON0
				movlw	chan7
				movwf	ADCON0
				call	measure

adcchan7:		btfsc	Registertemp3,.7		
				goto	adc7					
				btfsc	Registertemp1,.7		
				goto	adc7			
				goto	adc77
adc7:;			call	wr_zero
				bsf		pank1
				clrf	Register9
				clrf	Register9+.1
				bcf		pank1
				goto	adcchan8
adc77:;			nop
				banksel	ADCON0
				movlw	chan8
				movwf	ADCON0
				call	measure
adcchan8:		clrwdt
				goto	tsykkel
;===============================================================================
; **************' ADC-ga mõõtmine **********************************************
;===============================================================================
measure:		nop
				banksel	.0
				movlw	sampletime				; 277 uS
				movwf	sampledly
sdly:			decfsz	sampledly
				goto	sdly
				banksel	ADCON0
				bsf		ADCON0,GO
meas1:			BTFSC 	ADCON0,GO 
				goto	meas1
				BANKSEL ADRESH 
				MOVF 	ADRESH,W 
	banksel .0
				bsf		STATUS,IRP
				MOVWF 	INDF
				bcf		STATUS,IRP
				incf	FSR,F 
				BANKSEL ADRESL 
				MOVF 	ADRESL,W 
	banksel .0
				bsf		STATUS,IRP
				MOVWF 	INDF
				bcf		STATUS,IRP
				incf	FSR,F
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
Crc_Get_Bit:	rrf		mb_temp2,F				; bit in C
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
Crc_Shift:		rrf		_RS485chkH,F			; 16bit rotate
				rrf		_RS485chk,F
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
				bsf		pank1
				btfsc	Register273+.1,.7		; bit7 paarsus. 0=EVEN,1=NO PARITY
				goto	snd_exit
				bcf		pank1
; kalkuleeri paarsuse bitt
				movlw	.8
				movwf	countL
				clrf	adrtemp					; 1-tede loendi
				bcf		CARRY
parity:			rrf		countH,F
				btfsc	CARRY
				incf	adrtemp,F
				decfsz	countL
				goto	parity
				bcf		CARRY
				btfsc	adrtemp,.0
				bsf		CARRY
;				banksel	TXSTA					; paarsuse bitt on nüüd TXSTA,TX9D-s
			bsf		pank
				bcf		TXSTA,TX9D
				btfsc	CARRY
				bsf		TXSTA,TX9D

snd_exit:
		banksel	.0
snd_exita:		btfss   PIR1,TXIF     			; saatja valmis ?   
			    goto    snd_exita
				movf	Char,W;sendtemp,W				; saadetav bait meelde tuletada
				banksel	TXREG
				movwf   TXREG    				; saada!
				banksel	TXSTA
snd_exit1:		btfss	TXSTA,TRMT				; kas saatja nihkeregister tühi (bait prosest väljas)?
				goto	snd_exit1
				banksel	.0
;				bsf		pank
				movf	Char,W;sendtemp,W				; taasta saadetav
;				bcf		pank
			    return
;===============================================================================
				org		0x800
setup_serial:;	nop
				bsf		pank1
				movf	Register273+.1,W	
				andlw	0x07
				movwf	Register273
				bcf		pank1
				movlw	.71						; baudrate = (9600 @ 11.0592 MHz)
				banksel	SPBRG	
				movwf	SPBRG		
				banksel	.0
				bsf		pank1
				movf	Register273,W
				bcf		pank1
				sublw	.1						; kas on 9600 ?
				btfsc	ZERO
				goto	initser_par				; jah
				movlw	.35						; baudrate = (19200 @ 11.0592 MHz)
				banksel	SPBRG	
				movwf	SPBRG		
				banksel	.0
				bsf		pank1
				movf	Register273,W
				bcf		pank1
				sublw	.2						; kas on 19200 ?
				btfsc	ZERO
				goto	initser_par				; jah
				movlw	.17						; baudrate = (38400 @ 11.0592 MHz)
				banksel	SPBRG	
				movwf	SPBRG		
				banksel	.0
				bsf		pank1
				movf	Register273,W
				bcf		pank1
				sublw	.3						; kas on 38400 ?
				btfsc	ZERO
				goto	initser_par				; jah
				movlw	.11						; baudrate = (57600 @ 11.0592 MHz)
				banksel	SPBRG	
				movwf	SPBRG		
				banksel	.0
				bsf		pank1
				movf	Register273,W
				bcf		pank1
				sublw	.4						; kas on 57600 ?
				btfsc	ZERO
				goto	initser_par				; jah
				movlw	.5						; baudrate = (115200 @ 11.0592 MHz)
				banksel	SPBRG	
				movwf	SPBRG		
				banksel	.0
				bsf		pank1
				movf	Register273,W
				bcf		pank1
				sublw	.5						; kas on 115200 ?
				btfsc	ZERO
				goto	initser_par				; jah
setup_serial0:
				movlw	.35						; mingi kamm, võtab baudrate = (19200 @ 11.0592 MHz)
				banksel	SPBRG	
				movwf	SPBRG		
				banksel	.0
				bsf		pank1
				movf	Register273+.1,W		; parandame vea: võtab vaikimisi seriali seaded aga ei muuda debounce ja sticky bitte ! Samuti ei näpi Wiegandi bitte !!!
				bcf		pank1
				andlw	0x78
				bsf		pank1
				movwf	Register273
				bcf		pank1
				movlw	defaultserial
				andlw	0x07
				bsf		pank1
				addwf	Register273,W
				movwf	Register273+.1			
				bcf		pank1
initser_par:	bsf		pank1
				clrf	Register273				; selle solkisime ää, nüüd nulli
				movlw	B'01100111'				; paarsuse kalkuleerimine - eeldame: 9 bitine saade
				btfsc	Register273+.1,.7		; paarsus even (0) või puudub (1) ?
				movlw	B'00100110'				; paarsus puudub: 8 bitine saade
				bcf		pank1
				banksel	TXSTA
				movwf	TXSTA
				banksel	.0
				movlw	B'11010000'				; sama RCSTA jaoks: eeldame paarsust st. 9-bitist saadet
				bsf		pank1
				btfsc	Register273+.1,.7		
				movlw	B'10010000'			
				bcf		pank1
				banksel	RCSTA
				movwf	RCSTA
				banksel	.0
				return
;===============================================================================
;******************* EEPROMi funksioonid ***************************************
;===============================================================================
;===============================================================================
; **** Loeb parameetrid  EEPROMist ja kirjutab süsteemi ************************
;===============================================================================
Read_Setup:		movlw	LOW(e_ADR)				; loe oma modbussi aadress
				banksel	EEADR
				movwf	EEADR
				clrf	EEADRH
				call	Read_EEPROM					
				banksel	Register274	
				movwf	Register274+.1
				clrf	Register274
				call	Read_EEPROM				; loe oma ID (H)	
				banksel	Register274	
				movwf	Register258+.1
				clrf	Register258
				call	Read_EEPROM				; loe oma ID (L)	
				banksel	Register259	
				movwf	Register259+.1
				clrf	Register259
				call	Read_EEPROM				; Analoog-Pull-up'ide seis ehk anaväljundite seis	
				banksel	Register272	
				movwf	Register272+.1		
				banksel	Register0	
				movwf	Register0+.1			; ja väljundite seisu näitavasse registrisse (NB! ainult stardil)
				call	Read_EEPROM					
				banksel	Register272	
				movwf	Register272+.0			; Digital-Pull-up'ide seis
				banksel	Register0	
				movwf	Register0				; ja väljundite seisu näitavasse registrisse
			banksel	.0	
				movwf	DoutPort				; ja kohe väljunditesse kah (NB! ainult stardil)
				call	Read_EEPROM				; seriali parameetrid	
				banksel	Register273	
				movwf	Register273+.1
				clrf	Register273	
				banksel	.0
				movwf	Registertemp2			; süsteemile mugavamaks lugemiseks
				call	Read_EEPROM				; ANA-pordi (UIO) suund, 1= väljund, bitthaaval)	
				banksel	Register275	
				movwf	Register275+.1		
				banksel	.0
				movwf	Registertemp3			; süsteemile mugavamaks lugemiseks
				call	Read_EEPROM				; analoogpordi seisund stardil - analoog või digi. 1= digi
				banksel	Register271	
				movwf	Register271+.1
				clrf	Register271
				banksel	.0
				movwf	Registertemp1			; süsteemile mugavamaks lugemiseks
;				call	Read_EEPROM				; seadme tüüp	
;				banksel	Register274	
;				movwf	Register255+.1
;				clrf	Register255
				call	Read_EEPROM				; seadme tüüp	
				movwf	Register256+.1
				clrf	Register256
				call	Read_EEPROM				; firmware nr HIGH
				movwf	Register257+.0
				call	Read_EEPROM				; firmware nr LOW
				movwf	Register257+.1
				banksel	.0	
; *** resetid ****
				call	Read_EEPROM				; reset 1 ajad: viiteaeg peale pingestamist				
				banksel	Register276	
				movwf	Register276
				banksel	.0	
				movwf	reset1strttmrH
				call	Read_EEPROM					
				banksel	Register276	
				movwf	Register276+.1
				banksel	.0	
				movwf	reset1strttmrL
				call	Read_EEPROM				; reseti pulsi kestus				
				banksel	Register277	
				movwf	Register277
				banksel	.0	
				movwf	reset1pulsetmr
				call	Read_EEPROM					
				banksel	Register277	
				movwf	Register277+.1			; side kadumise viiteaeg
				banksel	.0
				movwf	reset1dlytmr
				call	Read_EEPROM				; reset 2 ajad: viiteaeg peale pingestamist				
				banksel	Register278	
				movwf	Register278
				banksel	.0	
				movwf	reset2strttmrH
				call	Read_EEPROM					
				banksel	Register278	
				movwf	Register278+.1
				banksel	.0	
				movwf	reset2strttmrL
				call	Read_EEPROM				; reseti pulsi kestus				
				banksel	Register279	
				movwf	Register279
				banksel	.0	
				movwf	reset2pulsetmr
				call	Read_EEPROM					
				banksel	Register279	
				movwf	Register279+.1			; side kadumise viiteaeg
				banksel	.0	
				movwf	reset2dlytmr
				call	Read_EEPROM				; juhitav aktiivne nivoo ANA-pordi sisendis
				banksel	Register271	
				movwf	Register271+.0
				banksel	.0	
				movwf	Registertemp4
				goto	setup_port				; kombineeri DO ja UIO juhtimine ja vastavate registrite sisud
				return
;===============================================================================
; **** Salvestab parameetrid EEPROMisse ja kirjutab süsteemi *******************
;===============================================================================
Save_Setup:		movlw	LOW(e_ADR)				; loe oma aadress
				banksel	EEADR
				movwf	EEADR
				clrf	EEADRH
				banksel	.0	
				bsf		pank1
				movf	Register274+.1,W		; aadress
				bcf		pank1
				call	Wr_EEPROM				; seivi
				banksel EEADR
				incf	EEADR,F					; järgmine aadress (ID-d ei kirjuta!)
				incf	EEADR,F					; järgmine aadress
				banksel	.0	
				bsf		pank1
				movf	Register272+.1,W		; PU-d - ANA pordile
				bcf		pank1
				call	Wr_EEPROM	
				bsf		pank1
				movf	Register272+.0,W		; PU-d DO - pordile
				bcf		pank1
				call	Wr_EEPROM
				call	setup_serial			; seriali seaded kohe serial porti
				bsf		pank1
				movf	Register273+.1,W		; seriali seaded uuesti sest kui oli viga, parandab pordi rutiin selle ära
				bcf		pank1
				movwf	Registertemp2			; süsteemile mugavamaks lugemiseks
				call	Wr_EEPROM				
				banksel	Register275
				movf	Register275+.1,W		; IOD ehk ANA-pordi suund
				banksel	.0
				movwf	Registertemp3		
				call	Wr_EEPROM				; kirjutame EEPROMi aga arvestame alles peale ANSELi sättimist !
				bsf		pank1
				movf	Register271+.1,W		; pordi Ana/Digi omadused (1=digi)
				bcf		pank1
				movwf	Registertemp1		
				call	Wr_EEPROM				; seivib EEPROMi
; *** resetid ****
				movlw	LOW(e_reset1)			; reset 1 ajad
				banksel	EEADR
				movwf	EEADR
				clrf	EEADRH
				banksel	.0	
				bsf		pank1
				movf	Register276,W			; viiteaeg peale pingestamist
				bcf		pank1
				movwf	reset1strttmrH
				call	Wr_EEPROM
				bsf		pank1
				movf	Register276+.1,W		
				bcf		pank1
				movwf	reset1strttmrL
				call	Wr_EEPROM
				bsf		pank1
				movf	Register277,W			; reseti pulsi kestus
				bcf		pank1
				movwf	reset1pulsetmr
				call	Wr_EEPROM
				bsf		pank1
				movf	Register277+.1,W		; side kadumise viiteaeg	
				bcf		pank1
				movwf	reset2dlytmr
				call	Wr_EEPROM
				movlw	LOW(e_reset2)			; reset 2 ajad
				banksel	EEADR
				movwf	EEADR
				clrf	EEADRH
				banksel	.0	
				bsf		pank1
				movf	Register278,W			; viiteaeg peale pingestamist
				bcf		pank1
				movwf	reset2strttmrH
				call	Wr_EEPROM
				bsf		pank1
				movf	Register278+.1,W		
				bcf		pank1
				movwf	reset2strttmrL
				call	Wr_EEPROM
				bsf		pank1
				movf	Register279,W			; reseti pulsi kestus
				bcf		pank1
				movwf	reset2pulsetmr
				call	Wr_EEPROM
				bsf		pank1
				movf	Register279+.1,W		; side kadumise viiteaeg	
				bcf		pank1
				movwf	reset2dlytmr
				call	Wr_EEPROM
				bsf		pank1
				movf	Register271+.0,W		; ANA-pordi väljundi aktiivse nivoo juhtimine
				bcf		pank1
				movwf	Registertemp4
				call	Wr_EEPROM
;===============================================================================
; kombineerib DO ja UIO juhtimise ja vastavate registrite sisud
;===============================================================================
setup_port:		comf	Registertemp1,W			; R271 -> pordi Ana/Digi omadused. 1 = DIGI
				banksel	ANSEL
				movwf	ANSEL					; prose ANSEL registrisse
				banksel	ANSELH
				clrf	ANSELH
				banksel	.0
				movf	Registertemp1,W			; sisendpinnile vastav bitt registris 1+1 (UIO sisendid) => 0
				banksel	Register1
				andwf	Register1+.1,F
				banksel .0
				movf	Registertemp3,W			; R275+.1 -> IOD ehk ANA-pordi suund. 1 = OUT
				banksel	ANSEL
				andwf	ANSEL,F					; kõik sisendid tuleb teha digiks sest analoogsisend annab alati 0.
				banksel	Registertemp3
				comf	Registertemp3,W			; prosel on 1 = IN aga kood tahab et olex vastupidi
				banksel	.0

				movwf	datatemp				; häälestab ANA-pordi suuna ringi IOD järgi: ana-port on pordis A ja E !
				bsf		datatemp,.7				; kvarts
				bsf		datatemp,.6				; kvarts
				bsf		datatemp,.5				; sisend 4
				btfsc	Registertemp3,.4
				bcf		datatemp,.5
				bsf		datatemp,.4				; vaba
				movf	datatemp,W
				banksel	TRISA
				movwf	TRISA					; pordi suund rakendatakse kohe: port A
				banksel	.0
				swapf	Registertemp3,W
				movwf	datatemp
				comf	datatemp,W
				movwf	datatemp
				rrf		datatemp,W
				andlw	0x07
				banksel	TRISE
				movwf	TRISE					; ja porti E
				banksel	.0
				bsf		pank1
				movf	Register0+.1,W			; taastame UIO väljundite seisu vastava registri järgi
				bcf		pank1
				movwf	Registertemp7
				andlw	0x0F
				movwf	PORTA					
				btfsc	Registertemp7,.4		; kombineerime baidid ANA-pordi juhtimisex	
				bsf		PORTA,.5
				bcf		CARRY
				rrf		Registertemp7,F
				swapf	Registertemp7,W
				andlw	0x07
				movwf	PORTE
				return
;setup_port:		movf	Registertemp3,W			; R275+.1 -> IOD ehk ANA-pordi suund. 1 = OUT
;				banksel	ANCON0
;				andwf	ANCON0,F				; kõik sisendid tuleb teha digiks sest analoogsisend annab alati 0.
;				movf	ANCON0,W
;				banksel	Registertemp3
;				comf	Registertemp3,W			; prosel on 1 = IN aga kood tahab et olex vastupidi
;				banksel	.0
;				movwf	datatemp				; häälestab ANA-pordi suuna ringi IOD järgi: ana-port on pordis A ja E !
;				bsf		datatemp,.7				; kvarts
;				bsf		datatemp,.6				; kvarts
;				bsf		datatemp,.5				; sisend 4
;				btfsc	Registertemp3,.4
;				bcf		datatemp,.5
;				bsf		datatemp,.4				; vaba
;				movf	datatemp,W
;				banksel	TRISA
;				movwf	TRISA					; pordi suund rakendatakse kohe: port A
;				banksel	Registertemp3
;				swapf	Registertemp3,W			; analoogpordi suund, 1= väljund, 0= sisend
;				movwf	datatemp
;				comf	datatemp,W
;				movwf	datatemp
;				bcf		CARRY
;				rrf		datatemp,W
;				andlw	0x07
;				banksel	TRISE
;				movwf	TRISE					; ja porti E
;				banksel	.0
;; pordi Ana/Digi omadused. 1 = DIGI, 0 = ANA
;				comf	Register275+.0,W		; 1=ANA: väljund ei saa olla analoog !
;				movwf	Registertemp7
;				comf	Register275+.1,W		; 0 = OUT
;				andwf	Registertemp7,W
;				banksel	ANCON0
;				movwf	ANCON0					; prose ANCONx registrisse
;				clrf	ANCON1
;				banksel	.0
;
;				movf	Register275+.0,W		; analoogpinnile vastav bitt registris 1+1 (UIO sisendid) => 0
;				banksel	Register1
;				andwf	Register1+.1,F
;				banksel .0
;
;				movff	Register0+.1,WREG		; taastame UIO väljundite seisu vastava registri järgi
;				andwf	Register275+.0,W		; maskeerime ANA/digi oamdusega (digi=1)
;				movwf	Registertemp7
;				andlw	0x0F
;				movwf	PORTA					
;				btfsc	Registertemp7,.4		; kombineerime baidid ANA-pordi juhtimisex	
;				bsf		PORTA,.5
;				bcf		CARRY
;				rrcf	Registertemp7,F
;				swapf	Registertemp7,W
;				andlw	0x07
;				movwf	PORTE
;				return
;===============================================================================
;  EEPROMi kirjutamine
;===============================================================================
Wr_EEPROM:	
				banksel EEDATA
				movwf	EEDATA
				banksel EECON1
				BCF 	EECON1,EEPGD
				BSF 	EECON1,WREN
				MOVLW 	55h
				MOVWF 	EECON2
				MOVLW 	0AAh
				MOVWF 	EECON2
				BSF 	EECON1,WR
wr_epr:			clrwdt
				BTFSC 	EECON1,WR
				GOTO 	wr_epr
				BCF 	EECON1,WREN
				banksel EEADR
				incf	EEADR,F					; järgmine aadress
				banksel EEDATA
				movf	EEDATA,W				; meenutame datat
				clrwdt
				banksel .0
				return
;===============================================================================
;  EEPROMist lugemine
;===============================================================================
Read_EEPROM:
				banksel EECON1
				BCF 	EECON1,EEPGD			; Point to DATA memory
				bsf		EECON1,RD			 	; loe!
				banksel	EEDATA
				movf	EEDATA,W
				incf	EEADR,F					; järgmine aadress
				return
;===============================================================================
; ******* I-nööbi funktsioonid *******
;===============================================================================
;===============================================================================
; ******* nööbi lugemine nulli *******
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
 			  clrf	    devicesfound	 ; esialgu pole ühtegi Dallase mölakat leitud ;)
             return
;===============================================================================
; ******* otsi nööpi *******
;===============================================================================
Search_1Wire: clrf      return_value     		; set return flag to false
              btfss     done_flag,0      		; Are we done yet?
              goto      Do_reset         		; No, start a search
              clrf      done_flag        		; Yes, init this for next time???
              goto      Wrapup           		; and get on out of here

Do_reset:     call      Reset_1wire				; nööbile saabast
              clrf      rombit_idx       		; set rom bit index to 1
              incf      rombit_idx, F
              clrf      curr_discrep     		; set descrepancy marker to 0
              movlw     0xF0            		; search rom käsk
              movwf     OneWireByte
              call      Sendbyte_1wire   		; saada käsk
Get_2bits             
              call      Readbit_1wire    		; read bit a from bus and
              movwf     bits             		; save it
              bcf		CARRY                  
              rlf       bits,F          		; shift bit A over to the left
              call      Readbit_1wire    		; read bit B from bus and 
              iorwf     bits,F			        ; save it

              movlw     HIGH lookup_1x
              movwf     PCLATH
              movf      bits,W

lookup_1x:    addwf     PCL, F           		; decode the bits read
              goto      bits_00          		; collision detected
              goto      bits_01          		; 0 read
              goto      bits_10          		; 1 read
              goto      bits_11          		; nobody there

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

              movlw     HIGH lookup_2x
              movwf     PCLATH
              movf      TEMP0,w

lookup_2x:    addwf     PCL, F           		; quik test for 0 or 1
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
              rrf       bits,W          		; get bit A into W
              call      Sendbit_1wire    		; send bit A to wire
; Increment rombit_idx
              incf      rombit_idx, F    		; bump pointer to next location
				clrwdt
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
Wrapup:		clrwdt
		       return
;===============================================================================
; ******* I-nööbi reset *******
;===============================================================================
Reset_1wire:	bcf 	_1WDAT               	; madalax
				call	wait					; 375 uS
				call	wait					; kokku ca. 750 uS
	            bsf 	_1WDAT	                ; kõrgex
				call	wait
				call	wait					; kokku ca. 750 uS
              	return
;===============================================================================
; ******* saadab käsubaidi registrist OneWireByte I-nööbile *******
;===============================================================================
Sendbyte_1wireA:movwf	OneWireByte				; seivi saadetav
Sendbyte_1wire:	movlw	0x08					; 8 bitti
              	movwf   TEMP0
_send_1w_lp:    clrf    TEMP1            		; Clear work area
              	bcf     TEMP1,0			        ; Assume sending 0
              	bcf		CARRY                   ; Fill new bits with 0
              	rrf     OneWireByte,F   		; Load CARRY with bit to send
              	btfsc   CARRY			        ; See what we are sending
               	bsf     TEMP1,0         		; must be a 1
              	movf    TEMP1,W		            ; Load up the bit and
              	call    Sendbit_1wire    		; send it out
              	decfsz  TEMP0,F         		; 8 bitti ok ?
               	goto     _send_1w_lp      		; vara veel
              	return
;===============================================================================
; ******* saadab käsubiti I-nööbile *******
;===============================================================================
Sendbit_1wire:  movwf   TEMP1
              	bcf     _1WDAT	         		; madalax
              	btfsc   TEMP1,0 
               	bsf     _1WDAT	        		; pulsi lõpp kui bitt oli 1
				movlw	.58						; (Tslot + Trec) - Tlow1
				call	wait_x
	            bsf     _1WDAT        			; pulsi lõpp kui bitt oli 0
              	nop    							; Trec
	            return
;===============================================================================
; ******* loeb vastusbaidi I-nööbist registrisse OneWireByte *******
;===============================================================================
Readbyte_1wire:	movlw   d'8'
               	movwf   TEMP0
_read_1w_lp:  	call    Readbit_1wire	   
              	movwf   TEMP1            		; Save the bit read
               	bcf		CARRY                   ; Assume bit is 0
              	btfsc   TEMP1,0         		; Check bit
               	bsf		CARRY                  	; its a 1
              	rrf     OneWireByte, F   		; Rotate bit into IO area
	            decfsz  TEMP0,F		 	        ; 8 bitti OK?
               	goto     _read_1w_lp      		; vara veel
              	movf    OneWireByte,W		    ; jah, tulemus W-sse
              	return                     ;
;===============================================================================
; ******* loeb vastusbiti I-nööbist *******
;===============================================================================
Readbit_1wire:  bcf    	_1WDAT	         		; taktipulss - madalax
				nop
	            bsf    	_1WDAT	         		; taktipulss - kõrgex
				bsf		pank					; lülitame nööbi otca sisendiks
				bsf		TRIS_1WDAT
				bcf		pank
	            clrf    TEMP1            		; Assume incomming bit is 0
				movlw	.12						; oota 12uS
				pagesel	wait_x
				call	wait_x
 	            btfsc   _1WDAT	         		; What are we receiving?
                bsf     TEMP1,0			        ; must be a 1
				movlw	.47						; oota 47uS
				call	wait_x
				pagesel	Readbit_1wire
              	movf    TEMP1,W
				bsf		pank					; lülitame nööbi otca väljundiks
				bcf		TRIS_1WDAT
				bcf		pank
	            return
;===============================================================================
; Store_Bit võtab bitt A (bits.1) ja salvestab alasse work, offset rombit_idx
; bits.1 -> work0..work7(rombit_idx)
;===============================================================================
Store_Bit:    call      SetupFSR         		; converdi rombit_idx mälu aadressiks
              movlw     HIGH SetWorkBit
              movwf     PCLATH            
              rrf       bits, W          		; loe bit.1 value right justified into W
              andlw     b'00000001'     		; lolle aadresse ei luba
              addwf     PCL, F           		; quick test for 0 or 1
              goto      ClrWorkBit       		; must be 0 (clr bit)
SetWorkBit                               		; must be 1 (set bit)
              clrc
              rlf       TEMP2, W         		; get bit position * 2 (0, 2, 4 .. 14)
              addwf     PCL, F           		; bump PC, turn bit on then return
              bsf       INDF, 7
              return
              bsf       INDF, 6
              return
              bsf       INDF, 5
              return
              bsf       INDF, 4
              return
              bsf       INDF, 3
              return
              bsf       INDF, 2
              return
              bsf       INDF, 1
              return
              bsf       INDF, 0
              return
ClrWorkBit
              movlw     HIGH ClrWorkBit
              movwf     PCLATH
              clrc
              rlf       TEMP2, W         		; get bit position * 2 (0, 2, 4 .. 14)
              addwf     PCL, F           		; bump PC and turn it off then return
              bcf       INDF, 7
              return
              bcf       INDF, 6
              return
              bcf       INDF, 5
              return
              bcf       INDF, 4
              return
              bcf       INDF, 3
              return
              bcf       INDF, 2
              return
              bcf       INDF, 1
              return
              bcf       INDF, 0
              return
;===============================================================================
;GetWorkBit -- rk0..work7(rombit_idx) -> W
;===============================================================================
GetWorkBit:   call      SetupFSR         		; viita soovitud bitile
              movlw     HIGH GetWorkBit
              movwf     PCLATH
              clrc
              rlf       TEMP2, W         		; get bit position * 2 into w
              addwf     TEMP2, W         		; compute w = bit offset * 3 (0, 3, 6 ...)
              addwf     PCL, F           		; bump PC to jump to appropriate test
;Bit 0
              btfss     INDF, 7          		; Is bit set(1)
               retlw    d'0'             		; No, return 0
              retlw     d'1'             		; Yes, return 1
;Bit 1
              btfss     INDF, 6         	 	; Is bit set(1)
               retlw    d'0'             		; No, return 0
              retlw     d'1'             		; Yes, return 1
;Bit 2
              btfss     INDF, 5          		; Is bit set(1)
               retlw    d'0'             		; No, return 0
              retlw     d'1'             		; Yes, return 1
;Bit 3
              btfss     INDF, 4          		;Is bit set(1)
               retlw    d'0'             		; No, return 0
              retlw     d'1'             		; Yes, return 1
;Bit 4
              btfss     INDF, 3          		;Is bit set(1)
               retlw    d'0'             		; No, return 0
              retlw     d'1'             		; Yes, return 1
;Bit 5
              btfss     INDF, 2          		;Is bit set(1)
               retlw    d'0'             		; No, return 0
              retlw     d'1'             		; Yes, return 1
;Bit 6
              btfss     INDF, 1          		;Is bit set(1)
               retlw    d'0'             		; No, return 0
              retlw     d'1'             		; Yes, return 1
;Bit 7
              btfss     INDF, 0          		; Is bit set(1)
               retlw    d'0'             		; No, return 0
              retlw     d'1'             		; Yes, return 1
;===============================================================================
; SetupFSR -- Point FSR at byte within work area determined by rombit_idx
;   Entry:
;         rombit_idx contians bit number (one relative) we need to get to
;   Exit:
;         FSR - pointing at byte within work0..work7
;         TEMP1 - zero relative byte offset (0,1,..7)
;         TEMP2 - zero relative(0,1,..7) bit offset within byte
;===============================================================================
SetupFSR                                 		; Setup FSR, TEMP1(byte offset), and TEMP2(bit offset)
              decf      rombit_idx, W    		; convert index to 0 relative
              sublw     d'63'            		; form compliment of rombit_idw 0-63->63-0
              movwf     TEMP1            		; and temp byte offset area
              andlw     b'00000111'      		; strip off byte offset leaving bit offset only
              movwf     TEMP2            		; and save into temp bit offset area
              
              bcf		CARRY            		; right justify the byte offset in TEMP1
              rrf       TEMP1, F         		; by right shifting
              bcf		CARRY                   ; and stripping 3 LSB's 
              rrf       TEMP1, F
              bcf		CARRY
              rrf       TEMP1, F
;Get FSR pointing at appropriate byte in work area (work0, work1 ...)
              movlw     work0            		;Point FSR at beginning of work area
              movwf     FSR
              movf      TEMP1,W            		;get byte offset (0 relative)
              addwf     FSR, F           		; point FSR straight at it
              return
;===============================================================================
; ******* reseti taimerite asjad *******
;===============================================================================
decrtmrs:		btfsc	reset1ena				; pingestamise viiteaeg juba läbi ?
				goto	T1intr1end				; jah
				movf	reset1strttmrH,W		; kas etteantud aeg = 0 ?
				addlw	.0
				btfss	ZERO
				goto	rtmr1notnull
				movf	reset1strttmrL,W
				addlw	.0
				btfsc	ZERO
				goto	rtmr1z					; jah, viiteaeg oli = 0, siis ei ootagi
rtmr1notnull:	decfsz	reset1strttmrL			; ei, pingestamisejärgne reset 1 viitetaimer alles käib
				goto	T1intr2
				movf	reset1strttmrH,W		; HIGH juba on  ?
				addlw	.0
				btfsc	ZERO
				goto	rtmr1z					; jah, siis aeg täis
				decf	reset1strttmrH,F
			decf	reset1strttmrL,F
				goto	T1intr2
rtmr1z:			bsf		reset1ena				; reseti 1 generaator on nüüd enableeritud
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
sv10:			bsf		pank1
				movf	Register277,W			; piisavalt IntereNetita oldud: 
				bcf		pank1
				movwf	reset1pulsetmr			; laeme pulsi kestuse
				addlw	.0
				btfsc	ZERO					; nulline kestus tähendab, et pulssi ei tekitatagi !
				goto	R1zero
				bsf		reset1pulseon			; märgime ära
				bcf		reset1					; ja nüüd annab saapaga ... !
			bsf		n_reset1					; inversioon
R1zero:			bsf		pank1
				movf	Register277+.1,W		; taasta side kadumise viiteaeg
				bcf		pank1
				movwf	reset1dlytmr
				goto	T1intr2					
T1intr1end1:	decfsz	reset1pulsetmr			; tixub pulsi kestust
				goto	T1intr2					
				bsf		reset1					; aitab kah pexmisest...
			bcf		n_reset1					; inversioon
				bcf		reset1pulseon			; märgime ära
				bsf		pank1
				movf	Register277,W			; laeme uuesti pulsi kestuse
				bcf		pank1
				movwf	reset1pulsetmr
			
T1intr2:		btfsc	reset2ena				; pingestamise viiteaeg juba läbi ?
				goto	T1intr2end				; jah
				movf	reset2strttmrH,W		; kas etteantud aeg = 0 ?
				addlw	.0
				btfss	ZERO
				goto	rtmr2notnull
				movf	reset2strttmrL,W
				addlw	.0
				btfsc	ZERO
				goto	rtmr2z					; jah, viiteaeg oli = 0, siis ei ootagi
rtmr2notnull:	decfsz	reset2strttmrL			; ei, pingestamisejärgne reset 2 viitetaimer alles käib
				return
				movf	reset2strttmrH,W		; HIGH juba on  ?
				addlw	.0
				btfsc	ZERO
				goto	rtmr2z					; jah, siis aeg täis
				decf	reset2strttmrH,F
			decf	reset2strttmrL,F
				return
rtmr2z:			bsf		reset2ena				; reseti 2 generaator on nüüd enableeritud
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

sv20:			bsf		pank1
				movf	Register279,W			; piisavalt IntereNetita oldud: laeme pulsi kestuse
				bcf		pank1
				movwf	reset2pulsetmr
				addlw	.0
				btfsc	ZERO					; nulline kestus tähendab, et pulssi ei tekitatagi !
				goto	R2zero
				bsf		reset2pulseon			; märgime ära
				bsf		PWRSW					; ja nüüd annab saapaga ... !

R2zero:			bsf		pank1
				movf	Register279+.1,W		; taasta side kadumise viiteaeg
				bcf		pank1
				movwf	reset2dlytmr

				return					
T1intr2end1:	decfsz	reset2pulsetmr			; tixub pulsi kestust
				return					
				bcf		PWRSW					; aitab kah pexmisest...
				bcf		reset2pulseon			; märgime ära
				bsf		pank1
				movf	Register279,W			; laeme uuesti pulsi kestuse
				bcf		pank1
				movwf	reset2pulsetmr
				pagesel saabas					; teeb endale reseti !!!
				goto	saabas
				return
;===============================================================================
; ******* Termoanduri asjad *******
;===============================================================================
;===============================================================================
; ******* käivitab temp. mõõtmise *******
;===============================================================================
Start_temp:		call	Reset_1wire
				movlw	SkipRom					; uus mõõtmine käima
				call	Sendbyte_1wireA
				movlw	ConvertT				
				call	Sendbyte_1wireA
				return
;===============================================================================
; ******* loeb andurilt temperatuuri näidu *******
;===============================================================================
GetTemp:		decfsz	_10sekunditmr
				goto	GetTemp_end
				movlw	_10sekundiaeg
				movwf	_10sekunditmr
				movf    devicesfound,W			; kas on üldse termoandureid ?
				addlw	.0
				btfsc	ZERO
				goto	GetTemp_end				; eip !
			bsf		temper
			goto	GetTemp_end				; mõõdab siis kui aega on

GetT:	bcf	INTCON,GIE
			bsf		_1WSPU					; tugev toide maha 
				clrwdt
;				pagesel	Reset_1wire
				call	Reset_1wire				; liinile saabast
				clrwdt                    		; WDT nullida 
;				pagesel	saada_adr
				call	saada_adr				; saada anduri aadress
				movlw	ReadScratch				; loe tulemust
;				pagesel	Sendbyte_1wireA
				call	Sendbyte_1wireA
				decf	CurrentDallas,W			; kalkuleerib anduri näidu aadressi
				movwf	work0
				bcf		CARRY
				rlf		work0,W			
				addlw	Dallas1
				movwf	FSR						; siin ta adre ongi
;				pagesel	Readbyte_1wirea
				call	Readbyte_1wirea			; loe temperatuur (LSB)
				movwf	work7					; seivi sest tahan hiljem saata  MSB esimesena
				call	Readbyte_1wirea			; MSB
;				pagesel	GetT
; siia võrdlus: kui >7d0 või F828 siis näitu ei uuendata
				movwf	work0
				sublw	0x07
				btfss	CARRY
				goto	Get_temp_dn
				movf	work0,W

				bsf		STATUS,IRP
				movwf	INDF
				bcf		STATUS,IRP
				incf	FSR,F
				movf	work7,W					; meenutame LSB-d
				bsf		STATUS,IRP
				movwf	INDF
				bcf		STATUS,IRP
Get_temp_dn:	call	Start_temp				; käivita uus mõõtmine
				bcf		_1WSPU					; tugev toide peale 
				decf	CurrentDallas,F			; 1 andur läbi käidud
				pagesel	GetT
				movf    CurrentDallas,W			; kõik ?
				addlw	.0
				btfss	ZERO
				goto	GetTemp_end				; eip !
				movf    devicesfound,W			; alustame algusest
				movwf	CurrentDallas
				bcf		temper					; mõõtmine tehtud
GetTemp_end:	
;	bsf	INTCON,GIE
				return
; *************************************************************************************************************
; ******************* DS1820 funktsioonid *********************************************************************
; *************************************************************************************************************
saada_adr:		movlw	MatchRom				; käsk vaid konkreetsele andurile
				call	Sendbyte_1wireA
				decf	CurrentDallas,W			; kalkuleerib anduri ID aadressi
				movwf	work0
				bcf		CARRY
				rlf		work0,W			
				movwf	work1
				rlf		work1,F			
				rlf		work1,W			
				addlw	Dallas1wa
	addlw .7
				movwf	FSR						; siin ta adre ongi
				movlw	.8						; seda on tervelt 8 baiti
				movwf	work3
saada_adr_loop: bsf		STATUS,IRP
				movf	INDF,W
				bcf		STATUS,IRP
				call	Sendbyte_1wireA			
			decf	FSR,F
				decfsz	work3
				goto	saada_adr_loop			; saadab kõik ID 8 baiti
				return
;===============================================================================
; ******* viited *******
;===============================================================================
wait:			movlw	0xFF
wait_x:			bsf		pank
				movwf	PR2
				bcf		pank
			movlw	0x01					; T2 periood 1uS, seisma!
			movwf	T2CON
				clrf	TMR2
				bcf		PIR1,TMR2IF
				bsf		T2CON,TMR2ON
wait_1:			clrwdt
				btfss	PIR1,TMR2IF
				goto	wait_1
				bcf		T2CON,TMR2ON
				bcf		PIR1,TMR2IF
				return
;===============================================================================
; ******* loeb vastusbaidi I-nööbist registrisse OneWireByte *******
;===============================================================================
Readbyte_1wirea:	movlw   d'8'
               	movwf   TEMP0
_read_1w_lpa:  	call    Readbit_1wirea	   
              	movwf   TEMP1            		; Save the bit read
               	bcf		CARRY                   ; Assume bit is 0
              	btfsc   TEMP1,0         		; Check bit
               	bsf		CARRY                  	; its a 1
              	rrf     OneWireByte, F   		; Rotate bit into IO area
	            decfsz  TEMP0,F		 	        ; 8 bitti OK?
               	goto     _read_1w_lpa      		; vara veel
              	movf    OneWireByte,W		    ; jah, tulemus W-sse
              	return                     ;
;===============================================================================
; ******* loeb vastusbiti I-nööbist *******
;===============================================================================
Readbit_1wirea:  bcf    	_1WDAT	         		; taktipulss - madalax
				nop
	            bsf    	_1WDAT	         		; taktipulss - kõrgex
				bsf		pank					; lülitame nööbi otca sisendiks
				bsf		TRIS_1WDAT
				bcf		pank
	            clrf    TEMP1            		; Assume incomming bit is 0
				movlw	.12						; oota 12uS
				pagesel	wait_y
				call	wait_y
 	            btfsc   _1WDAT	         		; What are we receiving?
                bsf     TEMP1,0			        ; must be a 1
				movlw	.47						; oota 47uS
				call	wait_y
				pagesel	Readbit_1wirea
              	movf    TEMP1,W
				bsf		pank					; lülitame nööbi otca väljundiks
				bcf		TRIS_1WDAT
				bcf		pank
	            return
wait_y:			bsf		pank
				movwf	PR2
				bcf		pank
			movlw	0x01					; T2 periood 1uS, seisma!
			movwf	T2CON
				clrf	TMR2
				bcf		PIR1,TMR2IF
				bsf		T2CON,TMR2ON
wait_1y:			btfss	PIR1,TMR2IF
				goto	wait_1y
				bcf		T2CON,TMR2ON
				bcf		PIR1,TMR2IF
				return
;===============================================
; sidetaimeri reload
;===============================================
reload_side:	bsf		sidetmron
				movlw	sidetime
				movwf	sidetaimer
				bsf		pank1
				movf	Register277,W			; taasta pulsi kestus
				bcf		pank1
				movwf	reset1pulsetmr		

				bsf		pank1
				movf	Register277+.1,W		; taasta side kadumise viiteaeg reseti 1 generaatorile
				bcf		pank1
				movwf	reset1dlytmr
				bsf		reset1					; reseti 1 pinn maha (igaks juhux)
				bcf		n_reset1				; inversioon

				bcf		reset1pulseon			; reseti 1 pulsi generaator OHV (igaks juhux)
rls1:			bsf		pank1
				movf	Register279+.0,W		; taasta pulsi kestus
				bcf		pank1
				movwf	reset2pulsetmr			

				bsf		pank1
				movf	Register279+.1,W		; side kadumise viiteaeg reseti 2 generaatorile
				bcf		pank1
				movwf	reset2dlytmr
				bcf		PWRSW					; reseti 2 pinn maha (igaks juhux)
				bcf		reset2pulseon			; reseti 2 pulsi generaator OHV (igaks juhux)
rls2:			return
;===============================================================================
; ******* initsialiseerimine *******
;===============================================================================
init:
				clrwdt                    		; WDT nullida 
				call	ajupesu					; mälu killimine

; **** WDT - vahipeni **** 
				clrwdt                    		; WDT nullida 
;; **** EEPORM **** 
;				banksel EECON1      		 
;				clrf	EECON1
; **** konf ****
				banksel OPTION_REG      		 
				movlw   b'10000111'         
				movwf   OPTION_REG      		; B-pullupid maha, 1:256 prescaler T0-le, sisemine takt, WPU lubatud 
; **** komparaatorid ****
				banksel CM1CON0      		 
				movlw 	0x00					; analoog komparaator välja
				movwf 	CM1CON0 
				movwf 	CM2CON0 
				movwf 	CM2CON1 
				banksel	VRCON
				clrf	VRCON
;--- A/D muundi ---
				BANKSEL ADCON1 
				MOVLW 	B'10000000' 			; right justify
				MOVWF 	ADCON1 					; Vdd and Vss as Vref
				BANKSEL ADCON0 
				MOVLW 	chan1		 			; ADC clock Fosc/32, AN0, On
				MOVWF 	ADCON0 
;--- setup, pordid --------						; loe setup
				banksel WPUB     		 
				movlw	0x00					; WPU-d maha
				movwf	WPUB
				banksel PORTA    		 
				movlw	0x00					; port sellisesse lähteseisu
				movwf 	PORTA 
				banksel TRISB     		 
				movlw	0xFF					; kõik digisisendid ja sisenditeks
				movwf 	TRISB 
				banksel PORTB    		 
				movlw	0xFF					; port sellisesse lähteseisu
				movwf 	PORTB 
				banksel TRISC     		 
			movlw	0xC0;8					; serial on sisend, DIR ja muud väljunditeks (v.a. 1Wire data otc ja C3 mis oli enne reset2)
				movwf 	TRISC 
				banksel PORTC    		 
				movlw	0xC7					; port sellisesse lähteseisu
				movwf 	PORTC 
				banksel TRISD     		 
				movlw	0x00					; kõik väljunditeks
				movwf 	TRISD 
				banksel PORTD    		 
				movlw	0x00					; port sellisesse lähteseisu
				movwf 	PORTD 
; bsf PWRSW
;				banksel TRISE     		 
;				movlw	0x0F					; analoogsisendid
;				movwf 	TRISE 
;				banksel PORTE    		 
				movlw	0x08					; port sellisesse lähteseisu
				movwf 	PORTE 
;				banksel	.0
				call	Read_Setup				; analoogkanali portide setup jms				
;--- USART -------------------- 				; USARTi häälestus (SYNC = 0, BRGH = 0, BRG16 = 1)
; Register 273
; b0,1,2 - kiirus, default 19200 (010)
; b3 -debounce kõigile sisenditele, default  ON
; b4 - sticky bitt kõigile sisenditele, default OFF
; b5,6 - Wiegand B,A lubatus
; b7 - paarsuse bitt, (0= even parity), default EVEN
				call	setup_serial
				pagesel	reset_ser1
				call	reset_ser1				; seriali reset, sidetaimeri reload
;--- põhitaimeri reload --------				; lae süsteemi põhitaimer
				movlw	T1resoL					; lae taimer 1 (katkestus iga 10 mS tagant)
				movwf	TMR1L
				movlw	T1resoH
				movwf	TMR1H
				movlw	0x31					; 1:8 prescaler, GO !
				movwf	T1CON					
;--- viitetaimeri reload --------				; lae Dallase viidete taimer
				movlw	0x01					; T2 periood 1uS, seisma!
				movwf	T2CON
				bsf		pank
				movlw	0xFF					; int peale 256 uS
				movwf	PR2
				bcf		PIE1,TMR2IE
				bcf		pank
				clrf	TMR2
				bcf		PIR1,TMR2IF
;--- pisike viide maha rahunemiseks ----
				movlw	.10						; rahuneme maha 1 s jooxul
				movwf	_10sekunditmr
relax:			movlw	0xFF
				movwf	sekunditmr
				pagesel	wait
relax1:			call	wait					; 375 uS
				clrwdt
				decfsz	sekunditmr
				goto	relax1
				decfsz	_10sekunditmr
				goto	relax
				clrwdt
				pagesel	relax
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
				movwf	IOCB					; lubame kõik pordi B muutuse katckestused
				banksel	.0
				bcf		INTCON,T0IE
;--- Variaablid ----------------				; muu pudi
				movf	DinPort,W
				movwf	dinpress
				comf	DinPort,W
				bsf		pank1
				movwf	Register1
				bcf		pank1
				movlw	0x00
				movwf	dinsticky
;				comf	AinPort,W
;				movwf	ainpress
				movlw	wiegandtime
				movwf	wiegandAtimer
				movwf	wiegandBtimer
				clrf	WAbitcount
				clrf	WBbitcount
				movlw	sekundiaeg
				movwf	sekunditmr
				movlw	0xFF
				movwf	lastinputs				; PORTB sisendite eelmine seis oli 0xFF			
;--- Dallase andurite avastamine ----------------
discovery:
				clrwdt                    		; WDT nullida 
				pagesel	Search_1Wire_init
       			call    Search_1Wire_init		; Dallase lugemine nulli
_search_next:
				clrwdt                    		; WDT nullida 
				pagesel	Search_1Wire
				call    Search_1Wire     		; käse otsida              
				pagesel	_search_next
            	btfss   return_value,0  		; Oli midagi ?
	            goto    _search_done     		; sitte mittagi...
				bcf		CARRY
				rlf		devicesfound,W			; Jess, oligi ! Seivime andurite tabelisse.
				movwf	_RS485chk
				rlf		_RS485chk,F			
				rlf		_RS485chk,W			
				addlw	Dallas1wa
				movwf	destination				; siia kirjutame
				movlw	work0	
				movwf	source					; siin on inff
				movlw	.8						; seda on tervelt 8 baiti
				movwf	countL
dl_mv_loop: 	movf	source,W
				movwf	FSR
				movf	INDF,W
				movwf	datatemp
				movf	destination,W
				movwf	FSR
				movf	datatemp,W
				bsf		STATUS,IRP
				movwf	INDF
				bcf		STATUS,IRP
				incf	destination,F
				incf	source,F
				decfsz	countL
				goto	dl_mv_loop
				incf	devicesfound,F			; loendame andureid
				movlw    MaxDallasSensors
				xorwf    devicesfound,W
				btfss    ZERO			        ; kas piir käes ?
				goto	_search_next			; eip, võta nekst Dallase mölakas
				clrwdt                    		; WDT nullida 
				call	Start_temp				; kõik andurid mõõtma ja kohe!
				bcf		_1WSPU					; tugev toide peale 
				movf    devicesfound,W			
				movwf	CurrentDallas
				movlw	sekundiaeg
				movwf	sekunditmr
_search_done:
; bcf PWRSW
				clrwdt                    		; WDT nullida 
				pagesel	main					; aitab küll, lõpteab selle porno ää
				return
;________________________________________________________
ajupesu:		movlw	0x21					; ajupesu
				movwf	FSR
				movlw	0x5F
				movwf	sidetaimer
				call	kill_mem
				movlw	0x10
				movwf	FSR
				movlw	0x6F
				movwf	sidetaimer
				call	kill_mem1
				movlw	0x90
				movwf	FSR
				movlw	0x5F
				movwf	sidetaimer
				call	kill_mem1
				return
kill_mem1:		movlw	0x00
				bsf		STATUS,IRP
				movwf	INDF
				bcf		STATUS,IRP
				incf	FSR,F
				decfsz	sidetaimer
				goto	kill_mem1
				return

kill_mem:		movlw	0x00
				movwf	INDF
				incf	FSR,F
				decfsz	sidetaimer
				goto	kill_mem
				clrf	sidetaimer
				return

saabas:	
				banksel	WDTCON
				bsf		WDTCON,SWDTEN
saabas1:		goto	saabas1
;===============================================================================
; ***************************** EEPROM *****************************************
;===============================================================================
; **** seadme ID ****
				org H'2100' 					; EEPROM'i sisu 
e_ADR:						de 0x01				; modbussi aadress
e_IDH:						de 0x82				; vidina unikaalne ID, bait 1
e_IDL:						de 0x01				; vidina unikaalne ID, bait 2
e_PUa:						de 0x00				; pullup mask ehk sisuliselt väljundite seis stardil sest PUsid juhitakse väljunditega
e_PUd:						de 0x00				; pullup mask ehk sisuliselt väljundite seis stardil sest PUsid juhitakse väljunditega
e_ser:						de 0x1A				; seriali ja sisendite lugemise parameetrid: vaata allpool
e_IOD:						de 0x00				; ANA-pordi (UIO) suund, 1= väljund, bitthaaval)
e_anadigi:					de 0xC0				; analoogpordi seisund stardil - analoog või digi. 1= digi
e_devtype					de 0xF1
e_firmwarenr				de 0x00,0x05		; F/w HIGH ja LOW
e_reset1:					de 0x02,0x58,0x05,0x1E		; 60 sekundit, 5 sekund, 30 sekundit
e_reset2:					de 0x02,0x58,0x05,0x64		; 600 sekundit, 5 sekund, 64 sekundit
e_xor:						de 0xFF				; sellega XORitakse ANA-pordi sisendit (et teha juhitav aktiivne HI/LO
; serialport:
;------------
; b0,1,2 - kiirus, default 19200 (010)
; b3 - debounce kõigile sisenditele, default: ON
; b4 - sticky bitt kõigile sisenditele, default: OFF
; b5 - wiegand B lubatud (PORTB,4 ja 5)
; b6 - wiegand A lubatud (PORTB,6 ja 7)
; b7 - paarsuse bitt, (0= even parity), default EVEN
	end			
;******************************************************************************
