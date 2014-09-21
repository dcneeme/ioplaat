 ;===============================================================================
; Universaalne IO-plaat modbus'i liinil. 8 digisisendit, 8 väljundit, 8 analoogsisendit
; Ftakt=11,0592 MHz.
; Side: 9600,n,8,2 modbus RTU
; 29.12.: lugemise, kirjutamise ja konfi käsud testitud, OK. Koopia siit !
; analoogtsükli parendamine, WR reg0 samuti. Koopia!
; 1.1.2013.: justkui toimiks. Koodikaitse ka peale. Neemele testimisele. Koopia !
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
;------------------------ LISA - WIEGAND --------------------
;mul ei ole muude pikkustega kui 26 kokkupuudet olnud. aga barix on oma x8 moodulis niimoodi teinud
;et koodi pikkus on eraldi registris 11 (aadress järelikult 10) kirjas. 
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
; 3.2.: kõik debouncega, sticky't ei ole. Aadressid alates 400-st. chk_len toimimise loogikat ei tea, lubab lugeda ja kõik!
; 22.2.: read_setup'i parandamine, reg 666 access teeb nüüd reseti.
; 27.2.: seriali kiiruse paranduse parandus, 3 väikest lisa. e_XOR peakx modb_write ja setupi puhul ka tulemuse enne porti kirjutamist xor-ima !
; 10.3.: register 767 teeb reseti, Enne oli 666 aga see jäi Dallase reg-te alla.
; 12.3.: Reg-te 271, 275 ja 0,1 toimimise täpsustamine. Koopia senisest.
; 28.3.: reset1 inversioonibitt PORTC,.3 lisatud.
; 6.4.: reload_side IGA baidi vastuvõtul ! Muudatused - CHG - Väidetalt tekib ajuti vastuse kadumine - uurida mispärast.
; 6.4.: side viga fiksed (vt. main juurest). Termo tõttu tekivad ajuti ikkagi viited. Vaja muuta termo algoritmi: katkestusest välja või state machine.
; 11.4.: termomeetreid pollib nüüd iga 10s tagant.
;---------------------------------------------------------
; 14.4.: uus prose - PIC18F46K80, alusta porteerimist...nüüd ! Reload_side iga minule antud käsu puhul ehk nagu alguses oli.
;||256||R||device type (F1)||
;||257||R||firmware serial number||
;||258||R||serial number 1, LSB||
;||259||R||serial number 2, LSB||

; 25.4.: võiks toimida - testimisele !
; 26.4.: fiksid, xyz juures võtab kirjutamise algaadressi vastuse jaoks valesti ! SEivi käsu lugemisel !
; 27.4.: Dallas toimib kah. Test jooxeb läbi - Neemele testida ! NB! Lisasin WDT perioodiga 4,2s.
; 29.4.: Vist ei jookse kokku nagu eelmine plaat. Ilma Dallaseta ei jäta ka vastuseid vahele.
; ajutine jõnx PWRSW-ga initis.
;
;--------------------------- Uus prose ja LISAME ka juhitava PWMi f-naalsuse -------------------------------
;kui aeglase pwm tegemiseks laheb, siis tasuks ehk kohe ka selle variandiga arvestada, et saaks kergesti lyhikeste impulssidega servo pwm teha.
;siis peaks kestuse ja perioodi juhtimine käima 100ms asemel ehk 100 mikrosekundi kaupa. yhiku valik kanali või perioodi registri yhe bitiga.
;juhtregistrid võiks aga nii globaalse perioodi kui kanalipulsside jaoks samad olla ja mingi vanem bit pulsi pikkkuse registris ytleks, mis yhikuid data 
;jaoks mõeldakse. servo pwm spetsialist ma ei ole, ehk helikopteritega seoses on sul rohkem aimu, mida seal vaja on.
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
;  bit10- välise synkro luba, 1=luba DI bit 15 0->1 alusel
;  bit9...bit0 - perioodi kestus, lubatud koodid 3..1023
;väline sünkro tähendaks seda, et mingi DI sisendi (naiteks DI bit15, kus katkestus oli) saaks kasutusele võtta 50Hz synkrosisendina. 
;saaks mitmekanalilise dimmeri teha... perioodi kestus tähistaks aga nominaalset sagedust, 50 või 60 Hz jaoks, või ka hoopis 100 vs 120 jaoks (pole ju oluline, 
;kumb poolperiood parajasti käimas). laevadele saaks pakkuda 400Hz seadmeid...
;lisaks välisele sünkrosisendile peaks olema synkroniseerimise võimalus modbus kaudu. uus perioodiarvestus peaks algama hetkel, kui reg 150 sisse midagi kirjutatakse.
;--------
;selline mõte tuli aga küll veel välise synkro kohta, et synkroimpulsi kasutamine ei peaks toimuma otse, katkestuse saamisel kohe loendeid nullistama tormates
;vaid läbi digitaalse faasidetektori (PLL). see võimaldaks tasahilju parandusi sisse viia ja synkroimpulsi ebatäpsest formeerimisest tulenevate juhuslike häirete 
;mõju vähendada (see viga on paljudel labastel dimmeritel).
;4 kanalit pwn on aga täiesti piisav.
;PLL puhul saab vist yhe registriga hakkama. võimalikud tähendused:
;kood 0 - synkro puudub
;kood 1 - sünkro sagedusele 50 Hz
;kood 2 - sünkro sagedusele 60 Hz
;
; PLL sisu on loendi, mis tegeleb synkro silumisega (keskmistamisega) - kuna yksikud
; synkroimpulsid võivad saabuda hajusalt, häirete ja anduri ebatäpsuse tõttu. selle 
; PLL loendi nullistumisi kasutataks siis impulsipikkustega tegelevate loendite juhtimisel. 
; see reversiivloendi oleks täpsuse huvides vähemalt 10-bitine ja loeks algseisust nullini maha.
; nullijõudmise ja välise synkro saabumise hetke vahe mõõdetakse, töödeldakse ja liidetakse
; loendi alglaadimiseks kasutatavale algseisu numbrile. see töötlus peaks kasutama proportsionaalset
; ja integreerivat vea arvestamist (käituma kui PI regulaator). juhtmõju=p*e+i*sum(e), kus e on
; faasiviga impulssides ja sum(e) on kogutud viga. veale e kui synkroimpulsi esifrondi ja loendi
; nullijõudmise vahele tuleb igaks juhuks lisada veel yks seaditustegur (faasinihe), mis kompenseerib
; synkroanduri hilistumise (või ettejõudmise). tegurid p ja i valitakse katseliselt selliselt, et
; saavutada kiire (minimaalse üleviskega) mittevõnkuv järelehäälestumine. optimumi leidmiseks on
; mitu metoodikat, sellega tegelen ise (tegurid peavad olema registrites, 1 bait kummalegi).
;
; muide, kuigi impulsi täiteteguri etteandmisel saaks lihtsalt aja anda, nagu yksikimpulsside korral,
; oleks pideva pwm jaoks mugavam kas ainsa või teise variandina kasutada juhtimises täitetegurit
; (siis ei sõltuks pikkuse arvestus valitud perioodist). muid perioode peale 50 ja 60 Hz jaoks vajalike
; ei pea synkroniseerima, aga võimalikud võiks olla isegi kuni 1 tunnini ulatuvad perioodid (põrandakütte juhtimine!).
;
; selle pll järgihäälestumise kirjeldamisega tekkiski juba üks uus mõte - yks variant PWM täiteteguri
; määramisel olla mitte lihtsalt täiteteguri etteandmine, vaid selle asemel mingit soovitud ja tegeliku
; suuruse etteandmine, mille vahet PWM väljund sisemise PI algoritmi kaudu nulli yritaks ajada. see
; vabastaks juhtkontrolleri PI-kontuuride juhtsuuruste pidevast arvutamisest ja meelespidamisest.
; iga kanali jaoks vaja siis ka oma p ja i ette anda, milleks tuleks registreid reserveerida.
;
; 5.5.: pwmi reg-e lugemine/kirjutamine tehtud, vist õige kah ;)
;================================================================
; setup_port ikka kummaline vist ? Uurida sealt alates !
; 12.5.: RCIE ja RCIF olid valedes registrites, fixed !
; 13.5.: termomeetrite näidu kontroll: kui 1* 85C, ei usu. Kui 2* järjest siis usub. Eelnevast juba: kui >125 -> ei usu
; justku toimib. Lisaks kontrollitud PWMi reg-te lugemist, ok.
; 6.6.: võtab vana plaadi koodist uptime-counteri koodi üle...(32-bitine uptime loendi reg. 498...499 (1F2, 1F3)) 
; teeme ka Dallase DS2438 kivi lugemise (kuni 9 kivi)
; F/w ver. 2.1
; 14.6.: reset on nüüd register 999 ! Teeme DS2438 registrite lugemist.
; 16.6.: ADC juhtbitid olid valed, nüüd Vref=4,1V ja muu kah õige.
; 23.6.: tehtud DS2438 lugemine, ok. Kivi tahab eraldi toidet ! F/w 2.2. NB ! TXIF tuleb keelata !!!
; 1.7.: mitmed parandused stabiilsuse nimel. Testime.... F/w 2.3.
; 2.7.: seni elus !!! Põhiline asi oli "bsf		PIE1,RC1IE" kohas "aaa". Testida termoandurite lugemise õigsust. Tegin RCIE1 lubamise ka sidetaimeri ületäitumise juurde !
; kui stardil DS-de otc mättas, jääbki ootama - fiksida ka siis kui töö ajal lühis tekib!
; seda ei viici fixida aga muu toimib. "Measure" juurde lisatud Taqu=1,5ms! F/w.: 2.4.
; 10.7.: "measure" juurde veel: kui ADC > 4095, ei salvesta!
; 14.7.: Ver. 2.5. Keskmistab kõiki ADC kanaleid, enne lahutab min ja max (üle 10 sämpli). Taqu=277uS tagasi !
; NB ! incrpointer pole vajalik, võiks välja visata! Kui viitcib...
; 4.8.: uus PWMi specs.
; 5.8.: T0 pulsi lõpetamine kompuleerub juba. Seiv.
; 6.8.: käskude viga fixed, vastab paremini, pulss tekib kuid periood vale !? T0int teha prioriteetseks (serial segab). Seiv.
; 8.8.: serint'i lõpus tuli sidetaimer seisata, solkis pika saadetise puhul läbi katkestuse counteri ää.
; Püüab teha T0inti prioriteetse... Tehtud aga segab - hakkab pakette vahele jätma
; 12.8.: PWM justqi töötab aga perioodi kestus on 1 sammu võrra liiga pikk ja pulsi kestus lühike. SEiv.
; 12.8.: pulsi kestused õiged. Seiv. Testima Neemele. Ver.: 2.6 ikka veel.
; 13.8.: termomeetrite lugemise viga fiksed - seriali puhvri äravõtmine nihutas lugemist. V.2.6 ikka !
; PWMi muutus kajastub ka registris 0. Register 149 kasutusele võetud. Vist toimib ?! Ver. 2.7 !!!
; ID-de lugemine oli nihu -fixed. Edasi teeb wr_multi vaid PWMi reg-tele (100...115). Koopia siit!
; wr-multi ka olemas, koopia. Ver. 2.8 !
; PWM nüüd hulka parem koos termodega. TEstimsele ! Ver. 2.9.
; 15.8.: chk_len asemel validate. Võib põhjustada clrsticky biti viga.. Fixed PWM'i juhtregistri 151 viga -> olid vahetuses. reset_pwm'is ka see viga. Testima. Ver.2.9. ikkagi.
; 20.8.: PWMi reset iga perioodi alguses EI tohi nullida faasiloendit! Fiksed. Ei ole nii, peab faasiloendi nullima. Viga mujal... :(
; 21.8.: viga oli selles et enne pulsi kestuse mahatixumist tuli kontrollida kas pulss üldse alanud. Koopia !
; 22.8.: pulss lõpeb siis kui on aeg mitte perioodi alguses. R0 ja pwm nagu seni, R151 on väljund ja r/o. Tehakse R0 ja PWMi XORina. Ver.: 2.10. Testima (perioodi kestus on vist veel veica vale (,25 us pikem !?)
; 24.8.: 1 pisuke viga fiksed...
; 25.8.: veel vigu fiksetud, PWM võix nüüd toimida. Lisaks sooviti et R273 kirjutamisel tehtax kohe reset1. Vist on.. ? Ver 2.11
; 26.8.: perioodi alguses ikkagi ei tohtinud kestusi taastada. 1ms loendi ei ole väga täpne, vist kuna kood aeglane. Testima...
; 27.8.: 1ms loendi ikkagi 4. Viga faasdnX kus Seero asemel kontrollliti ZERO't !. Neemele...
; 28.8.: faasup juures tuli enne lülitamist kontrollida, kas pulss äkki veel kestab. Testima...
; 1.9.: WDT reset oli 1h, nüüd ca 1s. RESET käsk vaid kui WR data DEAD. Discovery leiab DS2438d ka siis kui periood juba olemas (PEIE tuli keelata)
; 2.9.: ADC mõõtmisi parandet, incrpointer välja. On parem küll !
; väidetavalt ei vasta, minul vastab aga sitalt. WDT perioodi pikendada ? EI ! Viimase ADC knali tulemus kirjutas üle seriali puhvri 1.baidi. Mix???!!!???
; 4.9.: lubame 0 perioodi, siis toimivad üxixkpulsid. Testida !
; 9.9.: reset1 asemele TXi järgiv sisend mis tekitab Dir signaali. Dir lõpeb CRC arvutamise alguses ja muidugi seriali resetis.
; ei simuleeru (katckestusi ei tule)
; 28.9.: Viimane asi - C0 ehk 1WSPU püsti. Kurdeti et ilma anduriteta jääb PU peale. Ver.: 2.E
; 29.9.: DS2438 lugemine oli piiratud 8 anduriga. Peab olema 9.
; 6.10.: DS2438 aadressi rehkendamine vale - fiksed nüüd.
; 17.10.: katckestuste prioriteedi tõttu tuli ka discovery ajal katckestus ja asi tuxis. noints lipuga määrame kas 1W rutiinis katckestusi lubada või mitte. Versioon F. Ehk lõplik...
; 18.10.: 1W andurite loendamise piiri viga - oleks lugenud 18-ni. Fixed. Ver.: 0x10
; 19.10.: reset_pwm ei lülitanud TMR0 välja. Testida ! Ver 0x11. 
; 20.10.: TMR0 siiski pidi käima sest muidu ei toimi 1-impulsid. Stardil Keelasin PEIE, stardib stabiilsemalt küll! (v.: 0x02 0x11)
; 24.10.: viga reg 0, 1 lugemisel (mujal ka) - enne adr arvutamist puhvris (rlcf) tuli Cy nullida ! Ver 02, 12
; 26.10.: Reg150=0 kirjutamisel nullib nüüd õigesti kõik pulsid. Ver.: 2,14
; 27.10.: 1 pulsid töötavad ka siis kui per = 0. Kirjutamisel Reg150=0 ei tehta enam midagi erilist. Ver.: 2,15
; reset_pwm'is ei tohtinud _1mscount=4 teha kui per=0.
; toitekatckestuse peale kinni jooxmisel tuli EECON1 nullida ja siis uuesti oma aadress laadida. V 2,16
; lisasin EECON1=0 initisse, nii on õigem. Töötab !
; 28.10.: uued ettepanekud:
; 1) viia kogu ANA konf (in-out, ai-di) yhte registrisse 275.
; 2) jätta reg 271 yleni di inversiooni bitmapiks, nii DI kui ANA pordile
; 3) muuta di inversioonibittide tähendust registris 271, st mahatõmbamisel aktiivse oleku annaks bitiväärtus 1. siis on väljundi ja sisendi aktiivsused omavahel  vastavuses, mitte inverteeritud, nagu praegu.
; ANA konfi saaks aga yhte registrisse 275 nii:
; - ana-digi valikuks MSB, mis enne oli kasutamata
; - in-out valik jäägu LSB nagu praegu
; 29.10.: Tehtud, testima Neemele. Ver 2,17h. NB! Digi XOR mõjub vaid sisenditele aga mitte nt. wiegandile
; 30.10.: korduvpulsside nullimine. Ver 2,18. Testima! Etteantud kontrollkäskudega toimib. Perioodi piir millal nullitakse on 2.
; 31.10.: R271 MSB ja LSB vahetuses. Sisendeid loetakse inversiooniga. Fixed tehes comf ja siis xor (Ain_0, Din_0). V.: 2,19
; lisatud lugemise kaitsed. Ver sama.
; 27.12.: modb_write juures loendite aadress sai alati +1, fiksed (liita 164 mitte 165, read'is oli see juba parandet...) Ver.: 2, 1A
; 29.12.: kaitsebitid ei lubanud eepromi kirjutamist !
; 28.1.:2014: READ reg 273 teeb reseti ja annab vea - fixed: chk_alg5 juures. Saatmisel pidi jupitama _ jah, mõtleb tervelt 30us kuna prose on aeglane (paarsuse arvutus). Ver.: 2,1B
; muudatus reseti pulssides:
; praegu
; 276 RW reset1 (USB), toite pealetuleku kaitse aeg
; 277 RW reset1 (USB), pulsi kestus (MSB) ja rakendumise aeg (LSB)
; 278 RW reset2 (Phone Power Button?), toite pealetuleku kaitse aeg
; 279 RW reset2 (Phone Power Button?), pulsi kestus (MSB) ja rakendumise aeg (LSB)
;
;tulevikus
; 276 RW reset1 (USB), rakendumise aeg 16bit, sekundites
; 277 RW reset1 (USB), pulsi kestus 16 bit, sekundites
; 278 RW reset2 (Phone Power Button), rakendumise aeg 16bit, sekundites
; 279 RW reset2 (Phone Power Button), pulsi kestus 16 bit, sekundites
; toite pealetuleku kaitse aeg jääb ära !
; chk_alg5 juures: ei tee enam reset1-pulssi peale reg273 kirjutamist. Saab olema ver 2,1C
; modb_multikulti juures tuleb veel lubada loendite kirjutamine aga mitte hetkel...
; vigade parandus, ver 2,1D (31.1.2014)
; lisatud multikulti luba loenditele + fiks pwmi multikulti jaoks (sama ka loenditele) Ver.: 2,1E)
; selgus, et viited saates iga baidi tagant tekitab PWMi katckestus. Neeme mõtleb, mida teha...
; PWM iga 250 us tagant, kestus ca 180 us...teebki pausi ca 750us. Las mõtleb mis nüüd saab...
; 16.2. uued soovid: bootloader ja serial nr. register mitte kirjutatavaks.
; kokkuleppeline ploki pikkus võiks 128 baiti olla. siis mahub modbus sõnumisse kindlasti ära.
; oodatava koodi pikkuse saab bootloaderile teada anda, seerianr ja baitide arv sobib. sellele voiks lubatud ploki pikkusega vastata.
; peale iga ploki vastuvotmist voiks vastata vastuvoetud ploki jarjekorra numbriga, lubatud plokisuurust ja teavitatud mahtu arvestades - selle alusel saab ka master aru,
; kas upload edeneb edukalt voi on midagi vahele jaanud. vastuvoetud maht vastusena oleks kogu aeg yhesugune ega annaks infot, kas midagi kaduma laks.
; kui viimane plokk edukalt vastu voetud, siis on ploki number sellele vastav. ega siit edasi polegi rohkem midagi signaliseerida vaja, edasi peaks toimuma 
; mode registri muudatus ja reset.
; kui midagi laheb valesti - pikk timeout - siis hyppaks pic lihtsalt algseisu, kus ootab ser nr ja baitide arvu. eraldi errrorloaderit pole vaja. eelmise katse ebaedu
; kohta voib anda infot vastuvoetud ploki jarjekorranr registris vanima biti yleslykkamisega - siis saaab aru, mitmenda edukalt vastuvoetud ploki jarel pic arvates
; viga tekkis. registri algseis oleks 0.
;
; aga ainult loetav 32bit seerianumber on juba olemas, reg 258-259. hea oleks, kui seda yle programmeerida ei saaks, isegi kui tahaks.
; 17.2.: Reg 258,259 kirjutamine keelatud. Ver.: 2.20
; 18.2.: Reg 276,277 taastamise viga "R1zero" juures: pidi movff REgxxx->WREG mitte vastupidi ! Ver.: 2.21
; 19.2.: R258,9 kirjutamisel ei antud viga, nüüd annab (oleks pidanud ka enne andma !?). R276 default väärtus 00B4. ID = 0000.  V 2,22
; 19.2.: R276/7 ja 278/9 taastamise viga "reload_side's" fiksed. R258,9 viga chk_alg ütles et readonly. V2,23
; 13.3.: Ver. 2,24. CHG1 - peax vastama selle aadressiga mis saadeti ja osutus õigeks (v.a. broadcasti puhul). Nii saab aadressi muutmisel vastata senise aadressiga.
; 15.3.: tahetakse veel 8 loendit anasisenditest. Siis bootloaderit. Ver. 2,24, bootloaderit hetkel pole.
; 19.3.: Jobud !!! Ana loendid peaksid olemaseespool altes adr. 384 dec. Hetkel vist vastus lisaks 2 baiti nihkes paremale ? Näputööd alga ! Multikulti kirjutamisel oli viga - 1 baidi võrra nihkes. Kas see põhjustas PWMi imelikku käitumist ? V 2,25.
; 20.3.: lisaloendite r/w korras aga loendamine vale. Viga: lastinputs1 asemel ajuti lastinputs. V 2,26
; 23.3.: DIN nooremad loendid ei töötanud kuna neil polegi katkestusi. Ringi tehtud. A-in loendid töötasid ! V 2,27
; 24.3.; kui reseti viide anti 0, siis pulssi ei tekita. Ver.: 2,28
; 8.7.2014.: 1-juhe-andurite aadresside salvestamine ja lukustamine: tuleb sisse seada registriaadresside lukustamine registri 699 sisu kaudu. Kui sellel registriaadressil on 
; (vaike)väärtus 0, määratakse andurite järjekord iga restardi ajal uuesti. Kui selles on 1, kasutatakse viimase registriväärtuse 0 ajal toimunud restardil määratud järjekorda, 
; mida enam ei muudeta. Puuduvate (mittevastavate) temperatuuriandurite data on 4096, lisandunud järjekorras seni puuduvaid andureid ignoreeritakse.
; Lukustamine mõjutab kõiki 1-wire andureid, sh ka DS2438 registreid 700..785
; NB! peaks 699 lugemisel piirama vaid 1 registri peale !...
; 9.7.: väike fiks. Ver 2.29. Neemele testida.
; 15.7.: fiksitud, töötab (testitud). Neemele. Edasi teeme uut fiksi sisendite stardivea juures. Koopia !
; 15.7.: sisendite viga (stardil) fiksed - stardil laeti comf portb -> register 1. Ver.2.2A
; 18.7.: fiks DIN4..7 juures + eriti 4 juures. R276 algselt = 0 Ver 2,2B.
; 19.7.: fiks ei mõjunud st. loendi 4 loeb meeletult palju !?
; 22.7.: väidetavalt on viga siis kui debounce sees. Ilma on ok aga 4 esimest loendit ei tööta.
; loendite taimerikatckestuse (dinx_stk jne)juurest paar rida ää. Kas aitab ? in_debounce bitt nüüd teistpidi. Ver 2,2C.
; vaja veel tõsta lastinputsX biti nullimine veidi edasi kuni debounce tehtud. Ikka 2,2C
; Vahekoopia. Teeme kõik loendid ringi taimeri peale. sensXtmron ja sensXtmr-id võib välja visata kui vaja. Miskipärast ei toimi, tagasi!
; toimib küll kui vead ära parandada ! Ver.: 2,2C sest eelmised olid lihtsalt vigased!
; enne: initi lõppu veelkord "reset_ser1", nüüd vastab ka 1. päringule. Ver.: 2,2D
; 30.7.: proovime takti 4* üles pinistada... Ver.: 2,2E
; muudatused:
; - 		CONFIG	 PLLCFG = ON         	; Enabled
; - Taimer 0
;				movlw	B'01000101'				; 8 bitine, 1:64 prescaler T0-le, taimerina, seisma !
;				movwf	T0CON					
; - Taimer 1: T1reso=c9FF
; - Taimer 2: T2CON oli 0x01 -> 0x02 (prescaler 1:4 asemele 1:16)
; - Taimer 3: T3reso=E24C
; - A/D muundi: 
; 				MOVLW 	B'10111110' 			; right justified,-, 20 TAD, FOSC/64
;				MOVWF 	ADCON2 					
; - tema 				MOVLW 	B'10111110' 			; right justified,-, 20 TAD, FOSC/64
;				MOVWF 	ADCON2 					
; - measure juures kordame sdly rutiini 4 korda et saada kokku 278...us 
;				movlw	B'00100010'				; paarsus puudub: 8 bitine saade
;				banksel	TXSTA1
; 31.7.: call dly kah ära jäetud - polevat vaja
; - main juures:
;				movlw	B'00001000'				; RBIE int lubada, TMR0IE-d mitte !
; 31.7.: setup_ser juurde veel:
;;				movlw	B'01100111'				; paarsuse kalkuleerimine - eeldame: 9 bitine saade
;				movlw	B'01100011'				; paarsuse kalkuleerimine - eeldame: 9 bitine saade, SYNC=BRGH=BRG16=0
;				btfsc	Register273+.1,.7		; paarsus even (0) või puudub (1) ?
; Ver.: 2,2F Testida ! Eriti seda, kas yxik-pwm nüüd töötab.
; 4.8.: PWMi vea fiks (T0IE jms võeti mitu korda maha!).
; 5.8.: sendbit ja readbit rutiinid parandet - suudab kaablikeraga koos töötada. Ver 2,30.
; 5.8.: T0int lubatud alati. PWM toimib õigesti aga pulsi pikkus kõigub.
; 6.8.: pwmenabled välja. PWMi fiks (movff x,W ei ole sama mis x,WREG !) Ver 2,32. 1ms -> 0,75 jne  ?
; 6.8.: perioodi kvandiks taheti 1ms (250us asemel).T0reso=53h
; 10.8.: periood endiseks tagasi. Veel soove:
;1. kirjutades registrisse 999 koodi dc1a võiks toimuda 1wire discovery käivitamine, eeldusel et 699 sisu on 0.
;2. kirjutades 999 sisse koodi feed võiks käivituda reset1 hoolimata 276 sisust ja kestya vastavalt 277 poolt määratule.
;3. registrid 650... ja 750... võiks olla ka kirjutatavad, mis tähendab, et ka ilma discoverita on võimalik andurite asukohad paika panna. see on mugav siis, kui mingi andur tuleb vahetada voi midagi vaja lisada ja ei teha kõiki juba defineeritud andureid teise kohta saata. kirjutamine oleks lubatud 699 sisust sõltumata.
;4. kui saab, võiks mõlemat tüüpi 1w andurite arvu ka suurendada, kuni 16-ni. siis on viimane 1w id register 698 või 798.
; 14.8.: discovery ei jooxe enam kokku kui  liinil lühis ! Ver 2,33
; 14.8.: reg. 999 lisakäsud olemas ja justqi töötavad. Ver 2,33 ikkagi. Neemele testima
; 14.8.: punkt 3 ka tehtud. Simus töötab aga püüton vaid susiseb - vist on vigane. Neeme pusib...
; 17.8.: multikulti viga fiksed. Versioon ikka sama. Kirjutab eelmise vigase lihtsalt üle !
; 24.8.: saadetud 1W ID-d tuleb salvestada ka!
;===============================================================================
	include "P18F46K80.inc"  	
  	LIST   P=PIC18F46K80
;**** Prose konfi (ropud :) sõnad ****
;Program Configuration Register 1H
		CONFIG	 FOSC = HS1				; medium power osc
		CONFIG	 RETEN = OFF			; ULP regulator OFF
		CONFIG	 INTOSCSEL = HIGH 		; in Low-power mode during Sleep
		CONFIG	 SOSCSEL = DIG         	; Digital (SCLKI) mode
		CONFIG	 XINST = OFF          	; Disabled
		CONFIG	 PLLCFG = ON         	; Enabled
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
;        CONFIG	 CPB = ON     
;        CONFIG	 CPD = ON     
;		CONFIG	 WRT0 = ON   
;		CONFIG	 WRT1 = ON   
;		CONFIG	 WRT2 = ON   
;		CONFIG	 WRT3 = ON   
;		CONFIG	 WRTC = ON   
;		CONFIG	 WRTB = ON   
;		CONFIG	 WRTD = ON   
;				errorlevel -302					; Compaileri vingumised ära
;				errorlevel -305
;				errorlevel -306 
	errorlevel 0,-305,-302
;=============================================================================== 
;*********************** Raudvärk **********************************************
;*** Side ***
#define Dir				PORTC,5					; väljundpuhvri juhtimine, HIGH=rcv, LOW=trnsmt
#define TxD				PORTC,6					; seriali otc
#define RxD				PORTC,7					; seriali otc
#define RdRxD			PORTC,.2				; jälgib CCPIF kaudu seriali sidet ja tekitab sellele Dir signaali
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
;#define Din0			PORTB,.0				; digisisend 1
;#define Din1			PORTB,.1				; digisisend 2
;#define Din2			PORTB,.2				; digisisend 3
;#define Din3			PORTB,.3				; digisisend 4
;#define Din4			PORTB,.4				; digisisend 5
;#define Din5			PORTB,.5				; digisisend 6
;#define Din6			PORTB,.6				; digisisend 7
;#define Din7			PORTB,.7				; digisisend 8
#define Din0			Registertemp5,.0
#define Din1			Registertemp5,.1
#define Din2			Registertemp5,.2
#define Din3			Registertemp5,.3
#define Din4			Registertemp5,.4
#define Din5			Registertemp5,.5
#define Din6			Registertemp5,.6
#define Din7			Registertemp5,.7

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
;#define	reset1			PORTC,.2				; 3,3V reseti otc
#define	n_reset1		PORTC,.3				; 3,3V reseti inversioon-otc. Viimases versioonis sooviti uut inversiooni !
#define PWRSW			PORTC,.4				; mobla toitelüliti
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
;****** debounce ***
#define debtime			.1						; 10ms debounce aega igale sisendile
;****** side *******
#define RSBufLen			.99;				; saate-/vastuvõtu puhver
;****** käsud *****
#define modbus_rd			0x03				; käsk: loe registrit
#define modbus_wr			0x06				; käsk: kirjuta registrisse
#define modbus_wrmulti		0x10				; käsk: kirjuta mitu registrit korraga
#define modbus_cnf			0x45				; käsk: kirjuta konfiregistrisse
#define IllFunc				0x01				; veakood: keelatud käsk
#define IllDAdr				0x02				; veakood: vale registri aadress
#define maxregisters		.255;100+.170+.4				; niimitu 2-baidist registrit seadmel on
#define workregisters		.19+.17;0					; niimitu tööregistrit on, muud on kõrgema adrega ja süsteemi setupi jaoks
#define sidetime			.30					; 30 s möödudes kui ei laeku arusaadavaid käsk, võetakse default sideparameetrid
;****** taimerid, ajad ***
#define	serialtime			0x06				; 30 ms käsu laekumise aega alates 1. baidist
#define sekundiaeg			.100
#define _10sekundiaeg		.1;0
;#define	T1resoL				0x7F				; 10ms aeg @ 11.0592 MHz
;#define	T1resoH				0xF2					
#define	T1resoL				0xFF				; 10ms aeg @ 44.2368 MHz
#define	T1resoH				0xC9					
;#define	T3resoL				0x92				; 5,5ms aeg @ 11.0592 MHz
;#define	T3resoH				0xF8					
#define	T3resoL				0x4C				; 5,5ms aeg @ 44.2368 MHz
#define	T3resoH				0xE2					
#define	T0reso				.214;83;				; 250 us aeg @ 11.0592 MHz (ja nüüd ka @ 44,2368 MHz) Nüüd 1ms 0x83)
#define senstime			0x02;1				; 5 ms debounce aega
#define defaultserial		0x0A
#define	wiegandtime			.3;10					; 100 ms max ooteaega (viimasest pulsist loetuna)
; b0,1,2 - kiirus, default 19200 (010)
; b3 -debounce kõigile sisenditele, default  ON
; b4 - sticky bitt kõigile sisenditele, default OFF
; b5,6 - ei kasuta
; b7 - paarsuse bitt, (0= even parity), default EVEN
;****** Dallase kräpp *******
#define MaxDS1820Sensors	.9;*.2				; rohkem ei mahu mällu ää...
#define MaxDS2438Sensors	.9;*.2				; rohkem ei mahu mällu ää...
#define DS1820ID		0x28					; DS18B20 family code
#define DS2438ID		0x26					; DS2438 family code
#define	ConvertT		0x44					; käsk: mõõda temperatuuri (mõlematele anduritele)
#define	ConvertV		0xB4					; käsk: mõõda (pinget DS2438-le)
#define	SkipRom			0xCC					; käsk: ID-ROMi lugemine vahele 
#define	MatchRom		0x55					; käsk: loe vaid konkreetset andurit
#define	ReadScratch		0xBE					; käsk: loe temp. ja muu scratchpadilst
#define	RecallMem		0xB8					; käsk: kirjuta scratchpadi tulemused
#define WrScr			0x4E					; käsk: kirjuta scratchpad'i
#define _VddConnected	0x09
#define _VadcConnected	0x01
;************ andurid ************************
#define Akanaleid			.8					; niimitu kanalit on rauas olemas
#define mõõtmisi			.10					; 10 mõõtmist, neist min ja max lendavad välja
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

#define ainpress0			ainpress,.0			; analoogsisendite ajutised bitid
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

;#define sens9tmron			senstmrs1,.0
;#define sens10tmron			senstmrs1,.1
;#define sens11tmron			senstmrs1,.2
;#define sens12tmron			senstmrs1,.3
;#define sens13tmron			senstmrs1,.4
;#define sens14tmron			senstmrs1,.5
;#define sens15tmron			senstmrs1,.6
;#define sens16tmron			senstmrs1,.7


#define writedsID			flags,.0			; 1 kui multikultiga saadeti dallase ID ja see tuleks nüüd seivida
;#define special0			flags,.0
#define counters			flags,.1			; pöörduti loendite poole
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
#define loendi				flags1,.4
#define	pwm					flags1,.5			; pöördumine pwmi registrite poole
#define sync				flags1,.6			; kirjutati reg-e 150 mille peale syngitaxe PWMi
#define nomoreds2438		flags1,.7
#define Seero				flags1,.7			; PWMi pulsi lõpetamise juures
 
#define suurem				Measflags,.0
#define vaiksem				Measflags,.1
#define vordne				Measflags,.2
#define res_pwm				Measflags,.3
#define	write				Measflags,.4
#define	broadcast			Measflags,.5
#define noints				Measflags,.6
#define special1			Measflags,.7

#define special0			Measflags,.0
;**** 1-wire ****
#define DallasState			bits
;=============================================================================== 
; ************** Mälu jaotus **********	    	
				cblock 0x00
					sidetaimer					; mõõdab 30s side kadumise aega
					sekunditmr
					lastinputs					; PORTB sisendite eelmine seis
					lastinputs1		
					lastinputs2		
					flags
					flags1
					sampledly					; mõõtmise Taqu viide
					meascount					; analoogpingete keskmistamiste loendi
					Measflags					; min/max võrdlemise lipukesed
					serpartmp
; *** resettide taimerid ****
					reset1pulsetmrH
					reset1pulsetmrL
					reset1dlytmrH
					reset1dlytmrL
					reset2pulsetmrH
					reset2pulsetmrL
					reset2dlytmrH
					reset2dlytmrL
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
					muutus1
					senstmrs
;					senstmrs1
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
; ===== mälupank 2 =====
					phasecounter				; jooksva perioodi number
					RegX:.2
					AbiPuhver:.56;8;9;61		        ; esialgu vaba
					Register699:.2				; Kui on (vaike)väärtus 0, määratakse andurite järjekord iga restardi ajal uuesti.
; **** pwmi kirjutamise abi
					Register149in:.2			; Viide ms kuni järgmise perioodi alguseni
					Register149out:.2			; Aeg ms jooksva perioodi algusest
					pwmtmp:.3
; ===== mälupank 3 =====
; *** registrid ***
					Register0:.2				; digiväljundid (MSB) ja UIO (LSB), r/w	 		0xAB,AC NB! +1  siitmaalt alates
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
					Register270:.2				; r/w, LSB.0= pwmenabled, MSB:ADC Vref bitid 3,4,5 nagu PDF-is kirjas	0xDA,DB
; >>>CHG<<<										; PWMenabled enam ei arvesta !
					Register271:.2				; r/w, HIGH: DI pordi XOR, LOW: AI pordi XOR mask
; >>>CHG<<<
					Register272:.2				; r/w,  pull-upp'ide seis ehk DO seis stardil, 1= PU sees 0xDE,DF
					Register273:.2				; r/w, seriali parameetrid						0xE0,E1
					Register274:.2				; r/w,  modbus'i aadress						0xE2,E3
; >>>CHG<<<
					Register275:.2				; r/w, HIGH: UIO analoog/digitaalseis, 1=digisisend + LOW: analoogpordi UIO suund, 1= väljund, 0= sisend	0xE4,E5
; >>>CHG<<<
					Register276:.2				; r/w reset 1 toite pealetulemise kaitse aeg	0xE6,E7
					Register277:.2				; r/w reset 1 pulsi kestus (MSB) ja (LSB) 0xE8,E9
					Register278:.2				; r/w reset 2 toite pealetulemise kaitse aeg	0xEA,EB
					Register279:.2				; r/w reset 2 pulsi kestus (MSB) ja (LSB) 0xEC,ED
;					Register699:.2				; Kui on (vaike)väärtus 0, määratakse andurite järjekord iga restardi ajal uuesti.
;					Register999					; teeb reseti !
; *** temperatuuri kontroll ***
					DallasChk:.9				
 					DS2438Chk:.9				; kuni FF !!
				endc
; ===== mälupank 4 =====
; *** loendid ja Dallase kräpp ;) ***
				cblock 0x190					
; *** loendid ***
					Loendi9:.4					; AIN 1 pulsside loendi  Reg. 384,385
					Loendi10:.4					; 
					Loendi11:.4					; 
					Loendi12:.4					; 
					Loendi13:.4					; 
					Loendi14:.4					; 
					Loendi15:.4					; 
					Loendi16:.4					; 398,399
					Loendi1:.4					; DIN 1 pulsside loendi  Reg. 400,401
					Loendi2:.4					; 402, 403
					Loendi3:.4					; 404, 405
					Loendi4:.4					; 406, 407
					Loendi5:.4					; 408, 409
					Loendi6:.4					; 410, 411
					Loendi7:.4					; 412, 413
					Loendi8:.4					; 414, 415.
					Dallas1:.2					; Dallase termomeetrite näidud (Reg. 600 -> 47)
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
; bit15 – perioodiliselt korduv (1) või ühekordne (0) impulss, korduv=1
; bit14 – impulsi sidumine perioodi loendiga
; bit13..12 – faasiga sidumine, N*90 deg, N=1..3
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

;					Register149:.2				; kirjutamisel viide ms kuni järgmise perioodi alguseni. Lugemisel aeg ms jooksva perioodi algusest

; globaalne perioodi register, aadress 150, kus
;  bit15 - uuestikaivituse globaalne luba=1
;  bit14 - kestuse yhik, 100ms vs 100us? aeglasem=1
;  bit13..12  - reserv
;  bit10- välise synkro luba, 1=luba DI bit 15 0->1 alusel
;  bit9...bit0 - perioodi kestus, lubatud koodid 3..1023
; lisaks välisele sünkrosisendile peaks olema synkroniseerimise võimalus modbus kaudu.
; uus perioodiarvestus peaks algama hetkel, kui reg 150 sisse midagi kirjutatakse.
					Register150:.2				; register 150 PWMi periood 3…65535 ms (0x96)  ---> 0x320
; kuidas määratakse, kas antud väljund on PWMiga või tavaväljund/sisend
; tekitasin selleks eraldi bitmap registri, aadress naiteks 151, bitid 0-15
					Register151:.2				; register 151. Näitab väljundite hetkeseisu PWMi puhul
; *** PWMi tööregistrid ****
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
					periodtick:.3				; ühes perioodis on periood*4 sammu (250 us)
					masterpertick:.3			; sama aga siin numbrit hoitakse ja siit laetakse "periodtick'i"
					mphasetick:.2				; 1-s faasis täpselt 1*periood sammu
					phasetick:.2				; tööregister
					endc
; *** DS2438 registrid ***
					cblock	0x400				; Vadc, I, Vdd, Temp.
					DS2438_1:.8					; Dallase DS2438 kivi näidud: Reg. 700-2BC:(0x400,1), 701-2BD:(0x402,3), 702-2BE:(0x404,5), 703-2BF:(0x406,7)
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
; *** analoogpingete mõõtmine ja keskmistamine ***
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
 ; ************* asjad puhvris peale käsu vastuvõttu ****************	    	
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

#define 		in_sticky		Register273+.1,.4
#define 		in_debounce		Register273+.1,.3
; **** bootloaderile ****
#define 		BlBlockCount	dinflags

; **** serial port ****

; **** Prose lipukesed ****
#define CARRY           	STATUS,C 
#define ZERO            	STATUS,Z 
; *****************************	    	
   	
				org	0x000
;				goto	ErrorLoader
   				goto	main
;===============================================================================
;*********************** katckestused ******************************************
;===============================================================================
				org     0x0008					; Ints HIGH
;				goto	BootInts
				bsf		Dir						; Dir signaal algab
				bcf		PIE3,CCP2IE
				bcf		PIR3,CCP2IF
				retfie                   	

				org 	0x0018					; Ints LOW
_Push:
	    		movwf   W_Temp           		; Seivi kontekst        
				swapf   STATUS,W         
				movwf   S_Temp   
				movff	FSR0L,FSRtmpL
				movff	FSR0H,FSRtmpH
				banksel	.0
				btfsc	INTCON,RBIF				; kas mingi Wiegandi daataots või digisisend (loendi) liigutas ennast?
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
; ******* PWMi taimeri INT. *******
;===============================================================================
T0int:			bcf		INTCON,TMR0IF
				bcf		T0CON,TMR0ON			; taimer stopp kuniks tuunime
; bsf Dout7
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
; jobutamine läbi !!!
T0int1:			movff	periodtick+.0,WREG		; periood juba läbi ?
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
OutHigh:		; hakkame vastavalt vajadusele sobivaid väljundeid kõrgex tõstma 
Faasup0:		LFSR	.0,Register151+.1		; siia läheb kirja väljundi reaalne seis
				LFSR	.1,pwm0work+.0
				call	chkzero					; kas kestus on 0 ?
				btfsc	Seero
				goto	Faasup1					; jah, võta järgmine väljund
				btfss	INDF1,.6				; lülitame vaid sobival faasil ?
				goto	Faasup0nf				; eip, võib kohe lülitada
				swapf	INDF1,W					; jah, kas on õige faas ?
				andlw	0x03
				subwf	phasecounter,W		
				btfss	ZERO
				goto	Faasup1					; ei, võta järgmine väljund
Faasup0nf:		btfsc	RegX+.1,.0				; väljund läheb kõrgeks, kas juba on ?
				goto	Faasup1					; jah, võta järgmine väljund
				btfss	Register0+.1,.0			; väljundi määramiseks tee XOR reg-ga D0
				goto	Faasup0_0				; D0 on 0 -> väljund = 1
				bcf		PORTA,.0				
				bcf		INDF0,.0				; kajasta ka väljundite seisu r/o registris	151
				goto	Faasup0ex				; tehtud !
Faasup0_0:		bsf		PORTA,.0				
				bsf		INDF0,.0				; kajasta ka väljundite seisu r/o registris	151
Faasup0ex:		movff	pwm0set+.1,pwm0work+.1	; loe pulsi kestus (tarbetu ?)
				movff	pwm0set+.0,pwm0work+.0
				bsf		RegX+.1,.0
Faasup1:		LFSR	.1,pwm1work+.0
				call	chkzero					; kas kestus on 0 ?
				btfsc	Seero
				goto	Faasup2					; jah, võta järgmine väljund
				btfss	INDF1,.6				; lülitame vaid sobival faasil ?
				goto	Faasup1nf				; eip, võib kohe lülitada
				swapf	INDF1,W					; jah, kas on õige faas ?
				andlw	0x03
				subwf	phasecounter,W		
				btfss	ZERO
				goto	Faasup2					; ei, võta järgmine väljund
Faasup1nf:				btfsc	RegX+.1,.1				; väljund läheb kõrgeks, kas juba on ?
				goto	Faasup2					; jah, võta järgmine väljund
		btfss	Register0+.1,.1			; väljundi määramiseks tee XOR reg-ga D0
				goto	Faasup1_0				; D0 on 0 -> väljund = 1
;				btfsc	RegX+.1,.1				; väljund läheb kõrgeks, kas juba on ?
;				goto	Faasup2					; jah, võta järgmine väljund
				bcf		PORTA,.1				
				bcf		INDF0,.1				; kajasta ka väljundite seisu r/o registris	151
				goto	Faasup1ex				; tehtud !
Faasup1_0:		bsf		PORTA,.1				
				bsf		INDF0,.1				; kajasta ka väljundite seisu r/o registris	151
Faasup1ex:		movff	pwm1set+.1,pwm1work+.1	; loe pulsi kestus (tarbetu ?)
				movff	pwm1set+.0,pwm1work+.0
				bsf		RegX+.1,.1
Faasup2:		LFSR	.1,pwm2work+.0
				call	chkzero					; kas kestus on 0 ?
				btfsc	Seero
				goto	Faasup3					; jah, võta järgmine väljund
				btfss	INDF1,.6				; lülitame vaid sobival faasil ?
				goto	Faasup2nf				; eip, võib kohe lülitada
				swapf	INDF1,W					; jah, kas on õige faas ?
				andlw	0x03
				subwf	phasecounter,W		
				btfss	ZERO
				goto	Faasup3					; ei, võta järgmine väljund
Faasup2nf:				btfsc	RegX+.1,.2				; väljund läheb kõrgeks, kas juba on ?
				goto	Faasup3					; jah, võta järgmine väljund
		btfss	Register0+.1,.2			; väljundi määramiseks tee XOR reg-ga D0
				goto	Faasup2_0				; D0 on 0 -> väljund = 1
;				btfsc	RegX+.1,.2				; väljund läheb kõrgeks, kas juba on ?
;				goto	Faasup3					; jah, võta järgmine väljund
				bcf		PORTA,.2				
				bcf		INDF0,.2				; kajasta ka väljundite seisu r/o registris	151
				goto	Faasup2ex				; tehtud !
Faasup2_0:		bsf		PORTA,.2				
				bsf		INDF0,.2				; kajasta ka väljundite seisu r/o registris	151
Faasup2ex:		movff	pwm2set+.1,pwm2work+.1	; loe pulsi kestus (tarbetu ?)
				movff	pwm2set+.0,pwm2work+.0
				bsf		RegX+.1,.2
Faasup3:		LFSR	.1,pwm3work+.0
				call	chkzero					; kas kestus on 0 ?
				btfsc	Seero
				goto	Faasup4					; jah, võta järgmine väljund
				btfss	INDF1,.6				; lülitame vaid sobival faasil ?
				goto	Faasup3nf				; eip, võib kohe lülitada
				swapf	INDF1,W					; jah, kas on õige faas ?
				andlw	0x03
				subwf	phasecounter,W		
				btfss	ZERO
				goto	Faasup4					; ei, võta järgmine väljund
Faasup3nf:				btfsc	RegX+.1,.3				; väljund läheb kõrgeks, kas juba on ?
				goto	Faasup4					; jah, võta järgmine väljund
		btfss	Register0+.1,.3			; väljundi määramiseks tee XOR reg-ga D0
				goto	Faasup3_0				; D0 on 0 -> väljund = 1
;				btfsc	RegX+.1,.3				; väljund läheb kõrgeks, kas juba on ?
;				goto	Faasup4					; jah, võta järgmine väljund
				bcf		PORTA,.3				
				bcf		INDF0,.3				; kajasta ka väljundite seisu r/o registris	151
				goto	Faasup3ex				; tehtud !
Faasup3_0:		bsf		PORTA,.3				
				bsf		INDF0,.3				; kajasta ka väljundite seisu r/o registris	151
Faasup3ex:		movff	pwm3set+.1,pwm3work+.1	; loe pulsi kestus (tarbetu ?)
				movff	pwm3set+.0,pwm3work+.0
				bsf		RegX+.1,.3
Faasup4:		LFSR	.1,pwm4work+.0
				call	chkzero					; kas kestus on 0 ?
				btfsc	Seero
				goto	Faasup5					; jah, võta järgmine väljund
				btfss	INDF1,.6				; lülitame vaid sobival faasil ?
				goto	Faasup4nf				; eip, võib kohe lülitada
				swapf	INDF1,W					; jah, kas on õige faas ?
				andlw	0x03
				subwf	phasecounter,W		
				btfss	ZERO
				goto	Faasup5					; ei, võta järgmine väljund
Faasup4nf:				btfsc	RegX+.1,.4				; väljund läheb kõrgeks, kas juba on ?
				goto	Faasup5					; jah, võta järgmine väljund
		btfss	Register0+.1,.4			; väljundi määramiseks tee XOR reg-ga D0
				goto	Faasup4_0				; D0 on 0 -> väljund = 1
;				btfsc	RegX+.1,.4				; väljund läheb kõrgeks, kas juba on ?
;				goto	Faasup5					; jah, võta järgmine väljund
				bcf		PORTA,.5				
				bcf		INDF0,.4				; kajasta ka väljundite seisu r/o registris	151
				goto	Faasup4ex				; tehtud !
Faasup4_0:		bsf		PORTA,.5				
				bsf		INDF0,.4				; kajasta ka väljundite seisu r/o registris	151
Faasup4ex:		movff	pwm4set+.1,pwm4work+.1	; loe pulsi kestus (tarbetu ?)
				movff	pwm4set+.0,pwm4work+.0
				bsf		RegX+.1,.4
Faasup5:		LFSR	.1,pwm5work+.0
				call	chkzero					; kas kestus on 0 ?
				btfsc	Seero
				goto	Faasup6					; jah, võta järgmine väljund
				btfss	INDF1,.6				; lülitame vaid sobival faasil ?
				goto	Faasup5nf				; eip, võib kohe lülitada
				swapf	INDF1,W					; jah, kas on õige faas ?
				andlw	0x03
				subwf	phasecounter,W		
				btfss	ZERO
				goto	Faasup6					; ei, võta järgmine väljund
Faasup5nf:				btfsc	RegX+.1,.5				; väljund läheb kõrgeks, kas juba on ?
				goto	Faasup6					; jah, võta järgmine väljund
		btfss	Register0+.1,.5			; väljundi määramiseks tee XOR reg-ga D0
				goto	Faasup5_0				; D0 on 0 -> väljund = 1
;				btfsc	RegX+.1,.5				; väljund läheb kõrgeks, kas juba on ?
;				goto	Faasup6					; jah, võta järgmine väljund
				bcf		PORTE,.0				
				bcf		INDF0,.5				; kajasta ka väljundite seisu r/o registris	151
				goto	Faasup5ex				; tehtud !
Faasup5_0:		bsf		PORTE,.0				
				bsf		INDF0,.5				; kajasta ka väljundite seisu r/o registris	151
Faasup5ex:		movff	pwm5set+.1,pwm5work+.1	; loe pulsi kestus (tarbetu ?)
				movff	pwm5set+.0,pwm5work+.0
				bsf		RegX+.1,.5
Faasup6:		LFSR	.1,pwm6work+.0
				call	chkzero					; kas kestus on 0 ?
				btfsc	Seero
				goto	Faasup7					; jah, võta järgmine väljund
				btfss	INDF1,.6				; lülitame vaid sobival faasil ?
				goto	Faasup6nf				; eip, võib kohe lülitada
				swapf	INDF1,W					; jah, kas on õige faas ?
				andlw	0x03
				subwf	phasecounter,W		
				btfss	ZERO
				goto	Faasup7					; ei, võta järgmine väljund
Faasup6nf:				btfsc	RegX+.1,.6				; väljund läheb kõrgeks, kas juba on ?
				goto	Faasup7					; jah, võta järgmine väljund
		btfss	Register0+.1,.6			; väljundi määramiseks tee XOR reg-ga D0
				goto	Faasup6_0				; D0 on 0 -> väljund = 1
;				btfsc	RegX+.1,.6				; väljund läheb kõrgeks, kas juba on ?
;				goto	Faasup7					; jah, võta järgmine väljund
				bcf		PORTE,.1				
				bcf		INDF0,.6				; kajasta ka väljundite seisu r/o registris	151
				goto	Faasup6ex				; tehtud !
Faasup6_0:		bsf		PORTE,.1				
				bsf		INDF0,.6				; kajasta ka väljundite seisu r/o registris	151
Faasup6ex:		movff	pwm6set+.1,pwm6work+.1	; loe pulsi kestus (tarbetu ?)
				movff	pwm6set+.0,pwm6work+.0
				bsf		RegX+.1,.6
Faasup7:		LFSR	.1,pwm7work+.0
				call	chkzero					; kas kestus on 0 ?
				btfsc	Seero
				goto	Faasup8					; jah, võta järgmine väljund
				btfss	INDF1,.6				; lülitame vaid sobival faasil ?
				goto	Faasup7nf				; eip, võib kohe lülitada
				swapf	INDF1,W					; jah, kas on õige faas ?
				andlw	0x03
				subwf	phasecounter,W		
				btfss	ZERO
				goto	Faasup8					; ei, võta järgmine väljund
Faasup7nf:				btfsc	RegX+.1,.7				; väljund läheb kõrgeks, kas juba on ?
				goto	Faasup8					; jah, võta järgmine väljund
		btfss	Register0+.1,.7			; väljundi määramiseks tee XOR reg-ga D0
				goto	Faasup7_0				; D0 on 0 -> väljund = 1
;				btfsc	RegX+.1,.7				; väljund läheb kõrgeks, kas juba on ?
;				goto	Faasup8					; jah, võta järgmine väljund
				bcf		PORTE,.2				
				bcf		INDF0,.7				; kajasta ka väljundite seisu r/o registris	151
				goto	Faasup7ex				; tehtud !
Faasup7_0:		bsf		PORTE,.2				
				bsf		INDF0,.7				; kajasta ka väljundite seisu r/o registris	151
Faasup7ex:		movff	pwm7set+.1,pwm7work+.1	; loe pulsi kestus (tarbetu ?)
				movff	pwm7set+.0,pwm7work+.0
				bsf		RegX+.1,.7
; **** digiväljundite püstiajamine ****
Faasup8:		LFSR	.0,Register151+.0		; siia läheb kirja väljundi reaalne seis
				LFSR	.1,pwm8work+.0
				call	chkzero					; kas kestus on 0 ?
				btfsc	Seero
				goto	Faasup9					; jah, võta järgmine väljund
				btfss	INDF1,.6				; lülitame vaid sobival faasil ?
				goto	Faasup8nf				; eip, võib kohe lülitada
				swapf	INDF1,W					; jah, kas on õige faas ?
				andlw	0x03
				subwf	phasecounter,W		
				btfss	ZERO
				goto	Faasup9					; ei, võta järgmine väljund
Faasup8nf:		btfsc	RegX+.0,.0				; väljund läheb kõrgeks, kas juba on ?
				goto	Faasup9					; jah, võta järgmine väljund
				btfss	Register0+.0,.0			; väljundi määramiseks tee XOR reg-ga D0
				goto	Faasup8_0				; D0 on 0 -> väljund = 1
;				btfsc	RegX+.0,.0				; väljund läheb kõrgeks, kas juba on ?
;				goto	Faasup9					; jah, võta järgmine väljund
				bcf		Dout0				
				bcf		INDF0,.0				; kajasta ka väljundite seisu r/o registris	151
				goto	Faasup8ex				; tehtud !
Faasup8_0:		bsf		Dout0				
				bsf		INDF0,.0				; kajasta ka väljundite seisu r/o registris	151
Faasup8ex:		movff	pwm8set+.1,pwm8work+.1	; loe pulsi kestus (tarbetu ?)
				movff	pwm8set+.0,pwm8work+.0
				bsf		RegX+.0,.0
Faasup9:		LFSR	.1,pwm9work+.0
				call	chkzero					; kas kestus on 0 ?
				btfsc	Seero
				goto	Faasup10				; jah, võta järgmine väljund
				btfss	INDF1,.6				; lülitame vaid sobival faasil ?
				goto	Faasup9nf				; eip, võib kohe lülitada
				swapf	INDF1,W					; jah, kas on õige faas ?
				andlw	0x03
				subwf	phasecounter,W		
				btfss	ZERO
				goto	Faasup10				; ei, võta järgmine väljund
Faasup9nf:				btfsc	RegX+.0,.1				; väljund läheb kõrgeks, kas juba on ?
				goto	Faasup10				; jah, võta järgmine väljund
		btfss	Register0+.0,.1			; väljundi määramiseks tee XOR reg-ga D0
				goto	Faasup9_0				; D0 on 0 -> väljund = 1
;				btfsc	RegX+.0,.1				; väljund läheb kõrgeks, kas juba on ?
;				goto	Faasup10				; jah, võta järgmine väljund
				bcf		Dout1				
				bcf		INDF0,.1				; kajasta ka väljundite seisu r/o registris	151
				goto	Faasup9ex				; tehtud !
Faasup9_0:		bsf		Dout1				
				bsf		INDF0,.1				; kajasta ka väljundite seisu r/o registris	151
Faasup9ex:		movff	pwm9set+.1,pwm9work+.1	; loe pulsi kestus (tarbetu ?)
				movff	pwm9set+.0,pwm9work+.0
				bsf		RegX+.0,.1
Faasup10:		LFSR	.1,pwm10work+.0
				call	chkzero					; kas kestus on 0 ?
				btfsc	Seero
				goto	Faasup11				; jah, võta järgmine väljund
				btfss	INDF1,.6				; lülitame vaid sobival faasil ?
				goto	Faasup10nf				; eip, võib kohe lülitada
				swapf	INDF1,W					; jah, kas on õige faas ?
				andlw	0x03
				subwf	phasecounter,W		
				btfss	ZERO
				goto	Faasup11				; ei, võta järgmine väljund
Faasup10nf:				btfsc	RegX+.0,.2				; väljund läheb kõrgeks, kas juba on ?
				goto	Faasup11				; jah, võta järgmine väljund
		btfss	Register0+.0,.2			; väljundi määramiseks tee XOR reg-ga D0
				goto	Faasup10_0				; D0 on 0 -> väljund = 1
;				btfsc	RegX+.0,.2				; väljund läheb kõrgeks, kas juba on ?
;				goto	Faasup11				; jah, võta järgmine väljund
				bcf		Dout2				
				bcf		INDF0,.2				; kajasta ka väljundite seisu r/o registris	151
				goto	Faasup10ex				; tehtud !
Faasup10_0:		bsf		Dout2				
				bsf		INDF0,.2				; kajasta ka väljundite seisu r/o registris	151
Faasup10ex:		movff	pwm10set+.1,pwm10work+.1; loe pulsi kestus (tarbetu ?)
				movff	pwm10set+.0,pwm10work+.0
				bsf		RegX+.0,.2
Faasup11:		LFSR	.1,pwm11work+.0
				call	chkzero					; kas kestus on 0 ?
				btfsc	Seero
				goto	Faasup12				; jah, võta järgmine väljund
				btfss	INDF1,.6				; lülitame vaid sobival faasil ?
				goto	Faasup11nf				; eip, võib kohe lülitada
				swapf	INDF1,W					; jah, kas on õige faas ?
				andlw	0x03
				subwf	phasecounter,W		
				btfss	ZERO
				goto	Faasup12				; ei, võta järgmine väljund
Faasup11nf:				btfsc	RegX+.0,.3				; väljund läheb kõrgeks, kas juba on ?
				goto	Faasup12				; jah, võta järgmine väljund
		btfss	Register0+.0,.3			; väljundi määramiseks tee XOR reg-ga D0
				goto	Faasup11_0				; D0 on 0 -> väljund = 1
;				btfsc	RegX+.0,.3				; väljund läheb kõrgeks, kas juba on ?
;				goto	Faasup12				; jah, võta järgmine väljund
				bcf		Dout3				
				bcf		INDF0,.3				; kajasta ka väljundite seisu r/o registris	151
				goto	Faasup11ex				; tehtud !
Faasup11_0:		bsf		Dout3				
				bsf		INDF0,.3				; kajasta ka väljundite seisu r/o registris	151
Faasup11ex:		movff	pwm11set+.1,pwm11work+.1; loe pulsi kestus (tarbetu ?)
				movff	pwm11set+.0,pwm11work+.0
				bsf		RegX+.0,.3
Faasup12:		LFSR	.1,pwm12work+.0
				call	chkzero					; kas kestus on 0 ?
				btfsc	Seero
				goto	Faasup13				; jah, võta järgmine väljund
				btfss	INDF1,.6				; lülitame vaid sobival faasil ?
				goto	Faasup12nf				; eip, võib kohe lülitada
				swapf	INDF1,W					; jah, kas on õige faas ?
				andlw	0x03
				subwf	phasecounter,W		
				btfss	ZERO
				goto	Faasup13				; ei, võta järgmine väljund
Faasup12nf:				btfsc	RegX+.0,.4				; väljund läheb kõrgeks, kas juba on ?
				goto	Faasup13				; jah, võta järgmine väljund
		btfss	Register0+.0,.4			; väljundi määramiseks tee XOR reg-ga D0
				goto	Faasup12_0				; D0 on 0 -> väljund = 1
;				btfsc	RegX+.0,.4				; väljund läheb kõrgeks, kas juba on ?
;				goto	Faasup13				; jah, võta järgmine väljund
				bcf		Dout4				
				bcf		INDF0,.4				; kajasta ka väljundite seisu r/o registris	151
				goto	Faasup12ex				; tehtud !
Faasup12_0:		bsf		Dout4				
				bsf		INDF0,.4				; kajasta ka väljundite seisu r/o registris	151
Faasup12ex:		movff	pwm12set+.1,pwm12work+.1; loe pulsi kestus (tarbetu ?)
				movff	pwm12set+.0,pwm12work+.0
				bsf		RegX+.0,.4
Faasup13:		LFSR	.1,pwm13work+.0
				call	chkzero					; kas kestus on 0 ?
				btfsc	Seero
				goto	Faasup14				; jah, võta järgmine väljund
				btfss	INDF1,.6				; lülitame vaid sobival faasil ?
				goto	Faasup13nf				; eip, võib kohe lülitada
				swapf	INDF1,W					; jah, kas on õige faas ?
				andlw	0x03
				subwf	phasecounter,W		
				btfss	ZERO
				goto	Faasup14				; ei, võta järgmine väljund
Faasup13nf:				btfsc	RegX+.0,.5				; väljund läheb kõrgeks, kas juba on ?
				goto	Faasup14				; jah, võta järgmine väljund
		btfss	Register0+.0,.5			; väljundi määramiseks tee XOR reg-ga D0
				goto	Faasup13_0				; D0 on 0 -> väljund = 1
;				btfsc	RegX+.0,.5				; väljund läheb kõrgeks, kas juba on ?
;				goto	Faasup14				; jah, võta järgmine väljund
				bcf		Dout5				
				bcf		INDF0,.5				; kajasta ka väljundite seisu r/o registris	151
				goto	Faasup13ex				; tehtud !
Faasup13_0:		bsf		Dout5				
				bsf		INDF0,.5				; kajasta ka väljundite seisu r/o registris	151
Faasup13ex:		movff	pwm13set+.1,pwm13work+.1; loe pulsi kestus (tarbetu ?)
				movff	pwm13set+.0,pwm13work+.0
				bsf		RegX+.0,.5
Faasup14:		LFSR	.1,pwm14work+.0
				call	chkzero					; kas kestus on 0 ?
				btfsc	Seero
				goto	Faasup15				; jah, võta järgmine väljund
				btfss	INDF1,.6				; lülitame vaid sobival faasil ?
				goto	Faasup14nf				; eip, võib kohe lülitada
				swapf	INDF1,W					; jah, kas on õige faas ?
				andlw	0x03
				subwf	phasecounter,W		
				btfss	ZERO
				goto	Faasup15				; ei, võta järgmine väljund
Faasup14nf:					btfsc	RegX+.0,.6				; väljund läheb kõrgeks, kas juba on ?
				goto	Faasup15				; jah, võta järgmine väljund
	btfss	Register0+.0,.6			; väljundi määramiseks tee XOR reg-ga D0
				goto	Faasup14_0				; D0 on 0 -> väljund = 1
;				btfsc	RegX+.0,.6				; väljund läheb kõrgeks, kas juba on ?
;				goto	Faasup15				; jah, võta järgmine väljund
				bcf		Dout6				
				bcf		INDF0,.6				; kajasta ka väljundite seisu r/o registris	151
				goto	Faasup14ex				; tehtud !
Faasup14_0:		bsf		Dout6				
				bsf		INDF0,.6				; kajasta ka väljundite seisu r/o registris	151
Faasup14ex:		movff	pwm14set+.1,pwm14work+.1; loe pulsi kestus (tarbetu ?)
				movff	pwm14set+.0,pwm14work+.0
				bsf		RegX+.0,.6
Faasup15:		LFSR	.1,pwm15work+.0
				call	chkzero					; kas kestus on 0 ?
				btfsc	Seero
				goto	Faasup16				; jah, võta järgmine väljund
				btfss	INDF1,.6				; lülitame vaid sobival faasil ?
				goto	Faasup15nf				; eip, võib kohe lülitada
				swapf	INDF1,W					; jah, kas on õige faas ?
				andlw	0x03
				subwf	phasecounter,W		
				btfss	ZERO
				goto	Faasup16				; ei, võta järgmine väljund
Faasup15nf:				btfsc	RegX+.0,.7				; väljund läheb kõrgeks, kas juba on ?
				goto	Faasup16				; jah, võta järgmine väljund
		btfss	Register0+.0,.7			; väljundi määramiseks tee XOR reg-ga D0
				goto	Faasup15_0				; D0 on 0 -> väljund = 1
;				btfsc	RegX+.0,.7				; väljund läheb kõrgeks, kas juba on ?
;				goto	Faasup16				; jah, võta järgmine väljund
				bcf		Dout7				
				bcf		INDF0,.7				; kajasta ka väljundite seisu r/o registris	151
				goto	Faasup15ex				; tehtud !
Faasup15_0:		bsf		Dout7				
				bsf		INDF0,.7				; kajasta ka väljundite seisu r/o registris	151
Faasup15ex:		movff	pwm15set+.1,pwm15work+.1; loe pulsi kestus (tarbetu ?)
				movff	pwm15set+.0,pwm15work+.0
				bsf		RegX+.0,.7
Faasup16:		goto	T0intPulEnd				; jääb veel õige nati teha... :)
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
				movff	periodtick+.0,WREG		; periood sai nüüd läbi ?
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
				goto	T0intfaas				; sai läbi, alustame uut

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
				sublw	.4						; on max 4 faasi, üle selle pöörab algusesse tagasi
				btfss	ZERO
				goto	OutHigh
				movlw	0x00	
				movff	WREG,phasecounter
				goto	OutHigh

; ***** pulsi lõpetamine *****
T0intPulEnd:	decfsz	_1mscount				; 1 ms läbi ?
				goto	T0intend				; eip, siis ei tee midagi
T0intPulEnd1:	movlw	.4						; jaap, taasta 1ms loendi
				movwf	_1mscount
				LFSR	.0,Register151+.1		; kas antud väljundi pulss vaja lõpetada ?
FaasDn0:		LFSR	.1,pwm0work				; viita järgmisele väljundile
				call	chkzero					; jah, kas juba null (nt ei peagi pulssi olema) ?
				btfsc	Seero
				goto	FaasDn0Z				; ongi null ! Löö väljund ka nulli.
				btfss	RegX+.1,.0				; on pulss üldse alanud ?
				goto	FaasDn1					; eip, võta järgmine väljund
				call	decrcount				; ei ole veel 0, tixu kestust vähemaks
				call	chkzero					; kas nüüd on 0 ?
				btfss	Seero
				goto	FaasDn1					; eip, võta järgmine väljund
FaasDn0Z:		btfss	Register0+.1,.0			; reaalse väljundi määramiseks tee XOR reg-ga D0
				goto	FaasDn0_0				; D0 on 0 -> väljund = 0
				bsf		PORTA,.0				; D0 on 1 -> väljund = 1 
				bsf		INDF0,.0				; kajasta ka väljundite seisu r/o registris	151
				goto	Faasdn0ex				; tehtud !
FaasDn0_0:		bcf		PORTA,.0				; D0 on 0 -> väljund = 0 
				bcf		INDF0,.0				; kajasta ka väljundite seisu r/o registris	151
Faasdn0ex:		bcf		RegX+.1,.0				; märgi ära pwm'i (positiivse) loogika registrisse
				movff	pwm0set+.0,pwm0work+.0	; taasta pulsi kestus
				movff	pwm0set+.1,pwm0work+.1
				btfsc	INDF1,.7				; kui ühekordne pulss siis teeme edaspidi kestuse nullix
				goto	FaasDn1					; korduv - ei näpi rohkem !
				movff	pwm0work+.0,WREG		; ühekordne ja see sai nüüd tekitatud
				andlw	0xF0					; seega järgnevate pulsside kestus nullix
				movff	WREG,pwm0work+.0
				movlw	0x00
				movff	WREG,pwm0work+.1
FaasDn1:		LFSR	.1,pwm1work				; viita järgmisele väljundile
				call	chkzero					; jah, kas juba null (nt ei peagi pulssi olema) ?
				btfsc	Seero
				goto	FaasDn1Z				; ongi null ! Löö väljund ka nulli.
				btfss	RegX+.1,.1				; on pulss üldse alanud ?
				goto	FaasDn2					; eip, võta järgmine väljund
				call	decrcount				; ei ole veel 0, tixu kestust vähemaks
				call	chkzero					; kas nüüd on 0 ?
				btfss	Seero
				goto	FaasDn2					; eip, võta järgmine väljund
FaasDn1Z:		btfss	Register0+.1,.1			; reaalse väljundi määramiseks tee XOR reg-ga D0
				goto	FaasDn1_0				; D0 on 0 -> väljund = 0
				bsf		PORTA,.1				; D0 on 1 -> väljund = 1 
				bsf		INDF0,.1				; kajasta ka väljundite seisu r/o registris	151
				goto	Faasdn1ex				; tehtud !
FaasDn1_0:		bcf		PORTA,.1				; D0 on 0 -> väljund = 0 
				bcf		INDF0,.1				; kajasta ka väljundite seisu r/o registris	151
Faasdn1ex:		bcf		RegX+.1,.1				; märgi ära pwm'i (positiivse) loogika registrisse
				movff	pwm1set+.0,pwm1work+.0	; taasta pulsi kestus
				movff	pwm1set+.1,pwm1work+.1
				btfsc	INDF1,.7				; kui ühekordne pulss siis teeme edaspidi kestuse nullix
				goto	FaasDn2					; korduv - ei näpi rohkem !
				movff	pwm1work+.0,WREG		; ühekordne ja see sai nüüd tekitatud
				andlw	0xF0					; seega järgnevate pulsside kestus nullix
				movff	WREG,pwm1work+.0
				movlw	0x00
				movff	WREG,pwm1work+.1
FaasDn2:		LFSR	.1,pwm2work				; viita järgmisele väljundile
				call	chkzero					; jah, kas juba null (nt ei peagi pulssi olema) ?
				btfsc	Seero
				goto	FaasDn2Z				; ongi null ! Löö väljund ka nulli.
				btfss	RegX+.1,.2				; on pulss üldse alanud ?
				goto	FaasDn3					; eip, võta järgmine väljund
				call	decrcount				; ei ole veel 0, tixu kestust vähemaks
				call	chkzero					; kas nüüd on 0 ?
				btfss	Seero
				goto	FaasDn3					; eip, võta järgmine väljund
FaasDn2Z:		btfss	Register0+.1,.2			; reaalse väljundi määramiseks tee XOR reg-ga D0
				goto	FaasDn2_0				; D0 on 0 -> väljund = 0
				bsf		PORTA,.2				; D0 on 1 -> väljund = 1 
				bsf		INDF0,.2				; kajasta ka väljundite seisu r/o registris	151
				goto	Faasdn2ex				; tehtud !
FaasDn2_0:		bcf		PORTA,.2				; D0 on 0 -> väljund = 0 
				bcf		INDF0,.2				; kajasta ka väljundite seisu r/o registris	151
Faasdn2ex:		bcf		RegX+.1,.2				; märgi ära pwm'i (positiivse) loogika registrisse
				movff	pwm2set+.0,pwm2work+.0	; taasta pulsi kestus
				movff	pwm2set+.1,pwm2work+.1
				btfsc	INDF1,.7				; kui ühekordne pulss siis teeme edaspidi kestuse nullix
				goto	FaasDn3					; korduv - ei näpi rohkem !
				movff	pwm2work+.0,WREG		; ühekordne ja see sai nüüd tekitatud
				andlw	0xF0					; seega järgnevate pulsside kestus nullix
				movff	WREG,pwm2work+.0
				movlw	0x00
				movff	WREG,pwm2work+.1
FaasDn3:		LFSR	.1,pwm3work				; viita järgmisele väljundile
				call	chkzero					; jah, kas juba null (nt ei peagi pulssi olema) ?
				btfsc	Seero
				goto	FaasDn3Z				; ongi null ! Löö väljund ka nulli.
				btfss	RegX+.1,.3				; on pulss üldse alanud ?
				goto	FaasDn4					; eip, võta järgmine väljund
				call	decrcount				; ei ole veel 0, tixu kestust vähemaks
				call	chkzero					; kas nüüd on 0 ?
				btfss	Seero
				goto	FaasDn4					; eip, võta järgmine väljund
FaasDn3Z:		btfss	Register0+.1,.3			; reaalse väljundi määramiseks tee XOR reg-ga D0
				goto	FaasDn3_0				; D0 on 0 -> väljund = 0
				bsf		PORTA,.3				; D0 on 1 -> väljund = 1 
				bsf		INDF0,.3				; kajasta ka väljundite seisu r/o registris	151
				goto	Faasdn3ex				; tehtud !
FaasDn3_0:		bcf		PORTA,.3				; D0 on 0 -> väljund = 0 
				bcf		INDF0,.3				; kajasta ka väljundite seisu r/o registris	151
Faasdn3ex:		bcf		RegX+.1,.3				; märgi ära pwm'i (positiivse) loogika registrisse
				movff	pwm3set+.0,pwm3work+.0	; taasta pulsi kestus
				movff	pwm3set+.1,pwm3work+.1
				btfsc	INDF1,.7				; kui ühekordne pulss siis teeme edaspidi kestuse nullix
				goto	FaasDn4					; korduv - ei näpi rohkem !
				movff	pwm3work+.0,WREG		; ühekordne ja see sai nüüd tekitatud
				andlw	0xF0					; seega järgnevate pulsside kestus nullix
				movff	WREG,pwm3work+.0
				movlw	0x00
				movff	WREG,pwm3work+.1
FaasDn4:		LFSR	.1,pwm4work				; viita järgmisele väljundile
				call	chkzero					; jah, kas juba null (nt ei peagi pulssi olema) ?
				btfsc	Seero
				goto	FaasDn4Z				; ongi null ! Löö väljund ka nulli.
				btfss	RegX+.1,.4				; on pulss üldse alanud ?
				goto	FaasDn5					; eip, võta järgmine väljund
				call	decrcount				; ei ole veel 0, tixu kestust vähemaks
				call	chkzero					; kas nüüd on 0 ?
				btfss	Seero
				goto	FaasDn5					; eip, võta järgmine väljund
FaasDn4Z:		btfss	Register0+.1,.4			; reaalse väljundi määramiseks tee XOR reg-ga D0
				goto	FaasDn4_0				; D0 on 0 -> väljund = 0
				bsf		PORTA,.5				; D0 on 1 -> väljund = 1 
				bsf		INDF0,.4				; kajasta ka väljundite seisu r/o registris	151
				goto	Faasdn4ex				; tehtud !
FaasDn4_0:		bcf		PORTA,.5				; D0 on 0 -> väljund = 0 
				bcf		INDF0,.4				; kajasta ka väljundite seisu r/o registris	151
Faasdn4ex:		bcf		RegX+.1,.4				; märgi ära pwm'i (positiivse) loogika registrisse
				movff	pwm4set+.0,pwm4work+.0	; taasta pulsi kestus
				movff	pwm4set+.1,pwm4work+.1
				btfsc	INDF1,.7				; kui ühekordne pulss siis teeme edaspidi kestuse nullix
				goto	FaasDn5					; korduv - ei näpi rohkem !
				movff	pwm4work+.0,WREG		; ühekordne ja see sai nüüd tekitatud
				andlw	0xF0					; seega järgnevate pulsside kestus nullix
				movff	WREG,pwm4work+.0
				movlw	0x00
				movff	WREG,pwm4work+.1
FaasDn5:		LFSR	.1,pwm5work				; viita järgmisele väljundile
				call	chkzero					; jah, kas juba null (nt ei peagi pulssi olema) ?
				btfsc	Seero
				goto	FaasDn5Z				; ongi null ! Löö väljund ka nulli.
				btfss	RegX+.1,.5				; on pulss üldse alanud ?
				goto	FaasDn6					; eip, võta järgmine väljund
				call	decrcount				; ei ole veel 0, tixu kestust vähemaks
				call	chkzero					; kas nüüd on 0 ?
				btfss	Seero
				goto	FaasDn6					; eip, võta järgmine väljund
FaasDn5Z:		btfss	Register0+.1,.5			; reaalse väljundi määramiseks tee XOR reg-ga D0
				goto	FaasDn5_0				; D0 on 0 -> väljund = 0
				bsf		PORTE,.0				; D0 on 1 -> väljund = 1 
				bsf		INDF0,.5				; kajasta ka väljundite seisu r/o registris	151
				goto	Faasdn5ex				; tehtud !
FaasDn5_0:		bcf		PORTE,.0				; D0 on 0 -> väljund = 0 
				bcf		INDF0,.5				; kajasta ka väljundite seisu r/o registris	151
Faasdn5ex:		bcf		RegX+.1,.5				; märgi ära pwm'i (positiivse) loogika registrisse
				movff	pwm5set+.0,pwm5work+.0	; taasta pulsi kestus
				movff	pwm5set+.1,pwm5work+.1
				btfsc	INDF1,.7				; kui ühekordne pulss siis teeme edaspidi kestuse nullix
				goto	FaasDn6					; korduv - ei näpi rohkem !
				movff	pwm5work+.0,WREG		; ühekordne ja see sai nüüd tekitatud
				andlw	0xF0					; seega järgnevate pulsside kestus nullix
				movff	WREG,pwm5work+.0
				movlw	0x00
				movff	WREG,pwm5work+.1
FaasDn6:		LFSR	.1,pwm6work				; viita järgmisele väljundile
				call	chkzero					; jah, kas juba null (nt ei peagi pulssi olema) ?
				btfsc	Seero
				goto	FaasDn6Z				; ongi null ! Löö väljund ka nulli.
				btfss	RegX+.1,.6				; on pulss üldse alanud ?
				goto	FaasDn7					; eip, võta järgmine väljund
				call	decrcount				; ei ole veel 0, tixu kestust vähemaks
				call	chkzero					; kas nüüd on 0 ?
				btfss	Seero
				goto	FaasDn7					; eip, võta järgmine väljund
FaasDn6Z:		btfss	Register0+.1,.6			; reaalse väljundi määramiseks tee XOR reg-ga D0
				goto	FaasDn6_0				; D0 on 0 -> väljund = 0
				bsf		PORTE,.1				; D0 on 1 -> väljund = 1 
				bsf		INDF0,.6				; kajasta ka väljundite seisu r/o registris	151
				goto	Faasdn6ex				; tehtud !
FaasDn6_0:		bcf		PORTE,.1				; D0 on 0 -> väljund = 0 
				bcf		INDF0,.6				; kajasta ka väljundite seisu r/o registris	151
Faasdn6ex:		bcf		RegX+.1,.6				; märgi ära pwm'i (positiivse) loogika registrisse
				movff	pwm6set+.0,pwm6work+.0	; taasta pulsi kestus
				movff	pwm6set+.1,pwm6work+.1
				btfsc	INDF1,.7				; kui ühekordne pulss siis teeme edaspidi kestuse nullix
				goto	FaasDn7					; korduv - ei näpi rohkem !
				movff	pwm6work+.0,WREG		; ühekordne ja see sai nüüd tekitatud
				andlw	0xF0					; seega järgnevate pulsside kestus nullix
				movff	WREG,pwm6work+.0
				movlw	0x00
				movff	WREG,pwm6work+.1
FaasDn7:		LFSR	.1,pwm7work				; viita järgmisele väljundile
				call	chkzero					; jah, kas juba null (nt ei peagi pulssi olema) ?
				btfsc	Seero
				goto	FaasDn7Z				; ongi null ! Löö väljund ka nulli.
				btfss	RegX+.1,.7				; on pulss üldse alanud ?
				goto	FaasDn8					; eip, võta järgmine väljund
				call	decrcount				; ei ole veel 0, tixu kestust vähemaks
				call	chkzero					; kas nüüd on 0 ?
				btfss	Seero
				goto	FaasDn8					; eip, võta järgmine väljund
FaasDn7Z:		btfss	Register0+.1,.7			; reaalse väljundi määramiseks tee XOR reg-ga D0
				goto	FaasDn7_0				; D0 on 0 -> väljund = 0
				bsf		PORTE,.2				; D0 on 1 -> väljund = 1 
				bsf		INDF0,.7				; kajasta ka väljundite seisu r/o registris	151
				goto	Faasdn7ex				; tehtud !
FaasDn7_0:		bcf		PORTE,.2				; D0 on 0 -> väljund = 0 
				bcf		INDF0,.7				; kajasta ka väljundite seisu r/o registris	151
Faasdn7ex:		bcf		RegX+.1,.7				; märgi ära pwm'i (positiivse) loogika registrisse
				movff	pwm7set+.0,pwm7work+.0	; taasta pulsi kestus
				movff	pwm7set+.1,pwm7work+.1
				btfsc	INDF1,.7				; kui ühekordne pulss siis teeme edaspidi kestuse nullix
				goto	FaasDn8					; korduv - ei näpi rohkem !
				movff	pwm7work+.0,WREG		; ühekordne ja see sai nüüd tekitatud
				andlw	0xF0					; seega järgnevate pulsside kestus nullix
				movff	WREG,pwm7work+.0
				movlw	0x00
				movff	WREG,pwm7work+.1
; **** Digipordi väljundid ****
FaasDn8:		LFSR	.1,pwm8work				; viita järgmisele väljundile
				LFSR	.0,Register151+.0		; siia läheb kirja väljundi reaalne seis
				call	chkzero					; jah, kas juba null (nt ei peagi pulssi olema) ?
				btfsc	Seero
				goto	FaasDn8Z				; ongi null ! Löö väljund ka nulli.
				btfss	RegX+.0,.0				; on pulss üldse alanud ?
				goto	FaasDn9					; eip, võta järgmine väljund
				call	decrcount				; ei ole veel 0, tixu kestust vähemaks
				call	chkzero					; kas nüüd on 0 ?
				btfss	Seero
				goto	FaasDn9					; eip, võta järgmine väljund
FaasDn8Z:		btfss	Register0+.0,.0			; reaalse väljundi määramiseks tee XOR reg-ga D0
				goto	FaasDn8_0				; D0 on 0 -> väljund = 0
				bsf		Dout0					; D0 on 1 -> väljund = 1 
				bsf		INDF0,.0				; kajasta ka väljundite seisu r/o registris	151
				goto	Faasdn8ex				; tehtud !
FaasDn8_0:		bcf		Dout0					; D0 on 0 -> väljund = 0 
				bcf		INDF0,.0				; kajasta ka väljundite seisu r/o registris	151
Faasdn8ex:		bcf		RegX+.0,.0				; märgi ära pwm'i (positiivse) loogika registrisse
				movff	pwm8set+.0,pwm8work+.0	; taasta pulsi kestus
				movff	pwm8set+.1,pwm8work+.1
				btfsc	INDF1,.7				; kui ühekordne pulss siis teeme edaspidi kestuse nullix
				goto	FaasDn9					; korduv - ei näpi rohkem !
				movff	pwm8work+.0,WREG		; ühekordne ja see sai nüüd tekitatud
				andlw	0xF0					; seega järgnevate pulsside kestus nullix
				movff	WREG,pwm8work+.0
				movlw	0x00
				movff	WREG,pwm8work+.1
FaasDn9:		LFSR	.1,pwm9work				; viita järgmisele väljundile
				call	chkzero					; jah, kas juba null (nt ei peagi pulssi olema) ?
				btfsc	Seero
				goto	FaasDn9Z				; ongi null ! Löö väljund ka nulli.
				btfss	RegX+.0,.1				; on pulss üldse alanud ?
				goto	FaasDn10				; eip, võta järgmine väljund
				call	decrcount				; ei ole veel 0, tixu kestust vähemaks
				call	chkzero					; kas nüüd on 0 ?
				btfss	Seero
				goto	FaasDn10				; eip, võta järgmine väljund
FaasDn9Z:		btfss	Register0+.0,.1			; reaalse väljundi määramiseks tee XOR reg-ga D0
				goto	FaasDn9_0				; D0 on 0 -> väljund = 0
				bsf		Dout1					; D0 on 1 -> väljund = 1 
				bsf		INDF0,.1				; kajasta ka väljundite seisu r/o registris	151
				goto	Faasdn9ex				; tehtud !
FaasDn9_0:		bcf		Dout1					; D0 on 0 -> väljund = 0 
				bcf		INDF0,.1				; kajasta ka väljundite seisu r/o registris	151
Faasdn9ex:		bcf		RegX+.0,.1				; märgi ära pwm'i (positiivse) loogika registrisse
				movff	pwm9set+.0,pwm9work+.0	; taasta pulsi kestus
				movff	pwm9set+.1,pwm9work+.1
				btfsc	INDF1,.7				; kui ühekordne pulss siis teeme edaspidi kestuse nullix
				goto	FaasDn10				; korduv - ei näpi rohkem !
				movff	pwm9work+.0,WREG		; ühekordne ja see sai nüüd tekitatud
				andlw	0xF0					; seega järgnevate pulsside kestus nullix
				movff	WREG,pwm9work+.0
				movlw	0x00
				movff	WREG,pwm9work+.1
FaasDn10:		LFSR	.1,pwm10work			; viita järgmisele väljundile
				call	chkzero					; jah, kas juba null (nt ei peagi pulssi olema) ?
				btfsc	Seero
				goto	FaasDn10Z				; ongi null ! Löö väljund ka nulli.
				btfss	RegX+.0,.2				; on pulss üldse alanud ?
				goto	FaasDn11				; eip, võta järgmine väljund
				call	decrcount				; ei ole veel 0, tixu kestust vähemaks
				call	chkzero					; kas nüüd on 0 ?
				btfss	Seero
				goto	FaasDn11				; eip, võta järgmine väljund
FaasDn10Z:		btfss	Register0+.0,.2			; reaalse väljundi määramiseks tee XOR reg-ga D0
				goto	FaasDn10_0				; D0 on 0 -> väljund = 0
				bsf		Dout2					; D0 on 1 -> väljund = 1 
				bsf		INDF0,.2				; kajasta ka väljundite seisu r/o registris	151
				goto	Faasdn10ex				; tehtud !
FaasDn10_0:		bcf		Dout2					; D0 on 0 -> väljund = 0 
				bcf		INDF0,.2				; kajasta ka väljundite seisu r/o registris	151
Faasdn10ex:		bcf		RegX+.0,.2				; märgi ära pwm'i (positiivse) loogika registrisse
				movff	pwm10set+.0,pwm10work+.0; taasta pulsi kestus
				movff	pwm10set+.1,pwm10work+.1
				btfsc	INDF1,.7				; kui ühekordne pulss siis teeme edaspidi kestuse nullix
				goto	FaasDn11				; korduv - ei näpi rohkem !
				movff	pwm10work+.0,WREG		; ühekordne ja see sai nüüd tekitatud
				andlw	0xF0					; seega järgnevate pulsside kestus nullix
				movff	WREG,pwm10work+.0
				movlw	0x00
				movff	WREG,pwm10work+.1
FaasDn11:		LFSR	.1,pwm11work			; viita järgmisele väljundile
				call	chkzero					; jah, kas juba null (nt ei peagi pulssi olema) ?
				btfsc	Seero
				goto	FaasDn11Z				; ongi null ! Löö väljund ka nulli.
				btfss	RegX+.0,.3				; on pulss üldse alanud ?
				goto	FaasDn12				; eip, võta järgmine väljund
				call	decrcount				; ei ole veel 0, tixu kestust vähemaks
				call	chkzero					; kas nüüd on 0 ?
				btfss	Seero
				goto	FaasDn12				; eip, võta järgmine väljund
FaasDn11Z:		btfss	Register0+.0,.3			; reaalse väljundi määramiseks tee XOR reg-ga D0
				goto	FaasDn11_0				; D0 on 0 -> väljund = 0
				bsf		Dout3					; D0 on 1 -> väljund = 1 
				bsf		INDF0,.3				; kajasta ka väljundite seisu r/o registris	151
				goto	Faasdn11ex				; tehtud !
FaasDn11_0:		bcf		Dout3					; D0 on 0 -> väljund = 0 
				bcf		INDF0,.3				; kajasta ka väljundite seisu r/o registris	151
Faasdn11ex:		bcf		RegX+.0,.3				; märgi ära pwm'i (positiivse) loogika registrisse
				movff	pwm11set+.0,pwm11work+.0; taasta pulsi kestus
				movff	pwm11set+.1,pwm11work+.1
				btfsc	INDF1,.7				; kui ühekordne pulss siis teeme edaspidi kestuse nullix
				goto	FaasDn12				; korduv - ei näpi rohkem !
				movff	pwm11work+.0,WREG		; ühekordne ja see sai nüüd tekitatud
				andlw	0xF0					; seega järgnevate pulsside kestus nullix
				movff	WREG,pwm11work+.0
				movlw	0x00
				movff	WREG,pwm11work+.1
FaasDn12:		LFSR	.1,pwm12work			; viita järgmisele väljundile
				call	chkzero					; jah, kas juba null (nt ei peagi pulssi olema) ?
				btfsc	Seero
				goto	FaasDn12Z				; ongi null ! Löö väljund ka nulli.
				btfss	RegX+.0,.4				; on pulss üldse alanud ?
				goto	FaasDn13				; eip, võta järgmine väljund
				call	decrcount				; ei ole veel 0, tixu kestust vähemaks
				call	chkzero					; kas nüüd on 0 ?
				btfss	Seero
				goto	FaasDn13				; eip, võta järgmine väljund
FaasDn12Z:		btfss	Register0+.0,.4			; reaalse väljundi määramiseks tee XOR reg-ga D0
				goto	FaasDn12_0				; D0 on 0 -> väljund = 0
				bsf		Dout4					; D0 on 1 -> väljund = 1 
				bsf		INDF0,.4				; kajasta ka väljundite seisu r/o registris	151
				goto	Faasdn12ex				; tehtud !
FaasDn12_0:		bcf		Dout4					; D0 on 0 -> väljund = 0 
				bcf		INDF0,.4				; kajasta ka väljundite seisu r/o registris	151
Faasdn12ex:		bcf		RegX+.0,.4				; märgi ära pwm'i (positiivse) loogika registrisse
				movff	pwm12set+.0,pwm12work+.0; taasta pulsi kestus
				movff	pwm12set+.1,pwm12work+.1
				btfsc	INDF1,.7				; kui ühekordne pulss siis teeme edaspidi kestuse nullix
				goto	FaasDn13				; korduv - ei näpi rohkem !
				movff	pwm12work+.0,WREG		; ühekordne ja see sai nüüd tekitatud
				andlw	0xF0					; seega järgnevate pulsside kestus nullix
				movff	WREG,pwm12work+.0
				movlw	0x00
				movff	WREG,pwm12work+.1
FaasDn13:		LFSR	.1,pwm13work			; viita järgmisele väljundile
				call	chkzero					; jah, kas juba null (nt ei peagi pulssi olema) ?
				btfsc	Seero
				goto	FaasDn13Z				; ongi null ! Löö väljund ka nulli.
				btfss	RegX+.0,.5				; on pulss üldse alanud ?
				goto	FaasDn14				; eip, võta järgmine väljund
				call	decrcount				; ei ole veel 0, tixu kestust vähemaks
				call	chkzero					; kas nüüd on 0 ?
				btfss	Seero
				goto	FaasDn14				; eip, võta järgmine väljund
FaasDn13Z:		btfss	Register0+.0,.5			; reaalse väljundi määramiseks tee XOR reg-ga D0
				goto	FaasDn13_0				; D0 on 0 -> väljund = 0
				bsf		Dout5					; D0 on 1 -> väljund = 1 
				bsf		INDF0,.5				; kajasta ka väljundite seisu r/o registris	151
				goto	Faasdn13ex				; tehtud !
FaasDn13_0:		bcf		Dout5					; D0 on 0 -> väljund = 0 
				bcf		INDF0,.5				; kajasta ka väljundite seisu r/o registris	151
Faasdn13ex:		bcf		RegX+.0,.5				; märgi ära pwm'i (positiivse) loogika registrisse
				movff	pwm13set+.0,pwm13work+.0; taasta pulsi kestus
				movff	pwm13set+.1,pwm13work+.1
				btfsc	INDF1,.7				; kui ühekordne pulss siis teeme edaspidi kestuse nullix
				goto	FaasDn14				; korduv - ei näpi rohkem !
				movff	pwm13work+.0,WREG		; ühekordne ja see sai nüüd tekitatud
				andlw	0xF0					; seega järgnevate pulsside kestus nullix
				movff	WREG,pwm13work+.0
				movlw	0x00
				movff	WREG,pwm13work+.1
FaasDn14:		LFSR	.1,pwm14work			; viita järgmisele väljundile
				call	chkzero					; jah, kas juba null (nt ei peagi pulssi olema) ?
				btfsc	Seero
				goto	FaasDn14Z				; ongi null ! Löö väljund ka nulli.
				btfss	RegX+.0,.6				; on pulss üldse alanud ?
				goto	FaasDn15				; eip, võta järgmine väljund
				call	decrcount				; ei ole veel 0, tixu kestust vähemaks
				call	chkzero					; kas nüüd on 0 ?
				btfss	Seero
				goto	FaasDn15				; eip, võta järgmine väljund
FaasDn14Z:		btfss	Register0+.0,.6			; reaalse väljundi määramiseks tee XOR reg-ga D0
				goto	FaasDn14_0				; D0 on 0 -> väljund = 0
				bsf		Dout6					; D0 on 1 -> väljund = 1 
				bsf		INDF0,.6				; kajasta ka väljundite seisu r/o registris	151
				goto	Faasdn14ex				; tehtud !
FaasDn14_0:		bcf		Dout6					; D0 on 0 -> väljund = 0 
				bcf		INDF0,.6				; kajasta ka väljundite seisu r/o registris	151
Faasdn14ex:		bcf		RegX+.0,.6				; märgi ära pwm'i (positiivse) loogika registrisse
				movff	pwm14set+.0,pwm14work+.0; taasta pulsi kestus
				movff	pwm14set+.1,pwm14work+.1
				btfsc	INDF1,.7				; kui ühekordne pulss siis teeme edaspidi kestuse nullix
				goto	FaasDn15				; korduv - ei näpi rohkem !
				movff	pwm14work+.0,WREG		; ühekordne ja see sai nüüd tekitatud
				andlw	0xF0					; seega järgnevate pulsside kestus nullix
				movff	WREG,pwm14work+.0
				movlw	0x00
				movff	WREG,pwm14work+.1
FaasDn15:		LFSR	.1,pwm15work			; viita järgmisele väljundile
				call	chkzero					; jah, kas juba null (nt ei peagi pulssi olema) ?
				btfsc	Seero
				goto	FaasDn15Z				; ongi null ! Löö väljund ka nulli.
				btfss	RegX+.0,.7				; on pulss üldse alanud ?
				goto	FaasDn16				; eip, võta järgmine väljund
				call	decrcount				; ei ole veel 0, tixu kestust vähemaks
				call	chkzero					; kas nüüd on 0 ?
				btfss	Seero
				goto	FaasDn16				; eip, võta järgmine väljund
FaasDn15Z:		btfss	Register0+.0,.7			; reaalse väljundi määramiseks tee XOR reg-ga D0
				goto	FaasDn15_0				; D0 on 0 -> väljund = 0
				bsf		Dout7					; D0 on 1 -> väljund = 1 
				bsf		INDF0,.7				; kajasta ka väljundite seisu r/o registris	151
				goto	Faasdn15ex				; tehtud !
FaasDn15_0:		bcf		Dout7					; D0 on 0 -> väljund = 0 
				bcf		INDF0,.7				; kajasta ka väljundite seisu r/o registris	151
Faasdn15ex:		bcf		RegX+.0,.7				; märgi ära pwm'i (positiivse) loogika registrisse
				movff	pwm15set+.0,pwm15work+.0; taasta pulsi kestus
				movff	pwm15set+.1,pwm15work+.1
				btfsc	INDF1,.7				; kui ühekordne pulss siis teeme edaspidi kestuse nullix
				goto	FaasDn16				; korduv - ei näpi rohkem !
				movff	pwm15work+.0,WREG		; ühekordne ja see sai nüüd tekitatud
				andlw	0xF0					; seega järgnevate pulsside kestus nullix
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
				decf	POSTDEC1,F				; tixutame pulsi pikkust vähemax
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
; NB ! Sellel prosel pole B0..3 int-on-change omadust. Loendame taimeri katckestuses !
;===============================================================================
Loendid:		movf	PORTB,W
				movwf	dataport				; Wiegandi jaux
				xorwf	lastinputs,W			; mis muutus ?
				movwf	muutus
;>>>
				btfsc	Register273+.1,.5
				goto	loendid1
				btfsc	Register273+.1,.6
				goto	loendid1
				goto	Andur_end1				; kui Wiegand keelatud, siis ei töötle midagi
;>>>
				btfsc	muutus,.0				; loendi 1 ?
				goto	counter1				; jah
				btfsc	muutus,.1				; loendi 2 ?
				goto	counter2				; jah
				btfsc	muutus,.2				; loendi 3 ?
				goto	counter3				; jah
loendid1:		btfsc	muutus,.3				; loendi 4 ?
				goto	counter4				; jah
				btfsc	muutus,.4				; loendi 5 ?
				goto	counter5				; jah
				btfsc	muutus,.5				; loendi 6 ?
				goto	counter6				; jah
				btfsc	muutus,.6				; loendi 7 ?
				goto	counter7				; jah
			btfsc	muutus,.7				; loendi 8 ?
				goto	counter8				; jah
			goto	Andur_end				; ja kõik !
; *** Loendi 1 ***
counter1:		btfss	dataport,.0				; reageerime vaid langevale frondile
				goto	counter1_1
				bsf		lastinputs,.0			; oli tõusev front, updteerime sisendite eelmist seisu
				goto	Andur_end				; ja vaatame järgmisi sisendeid
counter1_1:		bcf		lastinputs,.0			; updateerime sisendite eelmist seisu
;				btfss	sens1tmron				; JUBA DEBOUNCEME ?
;				goto	counter1_2				; ei veel, hakkab pihta
				goto	Andur_end				; jah, vaatame järgmisi sisendeid
counter1_2:		bsf		sens1tmron 				; taimer käima
				goto	Andur_end				; ja kõik !
; *** Loendi 2 ***
counter2:		btfss	dataport,.1	
				goto	counter2_1
				bsf		lastinputs,.1
				goto	Andur_end	
counter2_1:		bcf		lastinputs,.1
;				btfss	sens2tmron	
;				goto	counter2_2	
				goto	Andur_end	
counter2_2:		bsf		sens2tmron 	
				goto	Andur_end	
; *** Loendi 3 ***
counter3:		btfss	dataport,.2	
				goto	counter3_1
				bsf		lastinputs,.2
				goto	Andur_end	
counter3_1:		bcf		lastinputs,.2
;				btfss	sens3tmron	
;				goto	counter3_2	
				goto	Andur_end	
counter3_2:		bsf		sens3tmron 	
				goto	Andur_end	
; *** Loendi 4 ***
counter4:		btfss	dataport,.3	
				goto	counter4_1
				bsf		lastinputs,.3
				goto	Andur_end	
counter4_1:		bcf		lastinputs,.3
;				btfss	sens4tmron	
;				goto	counter4_2	
				goto	Andur_end	
counter4_2:		bsf		sens4tmron 	
				goto	Andur_end	
; *** Loendi 5 või Wiegand B biti 0 sisend ***
counter5:; b5 - wiegand B lubatud (PORTB,4 ja 5)
; b6 - wiegand A lubatud (PORTB,6 ja 7)
				btfsc	Register273+.1,.5
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
; *** Loendi 7 või Wiegand A biti 0 sisend ***
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
; *** Loendi 7 või Wiegand A biti 1 sisend ***
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
Wiegand_Ahigh:
				banksel	PIE1
				bcf		PIE1,RC1IE
				banksel	.0
				btfsc	dataport,.7				; loeme vaid - fronti
				goto	wieg_endAH				; oli + front
				bcf		lastinputs,.7			; updateeri sisendite seisu
				btfsc	wiegandAto				; kui timeout siis on pakett juba käes ja loeme edasi vaid siis kui master on data ära lugenud
				goto	Andur_end				; minema siit !
				call	wieg_prepA				; + front, käivita paketi lõpu taimer
				call	rotH					; nihuta biti 1 väärtus registrisse
				incf	WAbitcount,F			; loendame bitte
				goto	Andur_end				; aitab !
Wiegand_Alow:
				banksel	PIE1
				bcf		PIE1,RC1IE
				banksel	.0
				btfsc	dataport,.6				; loeme vaid - fronti
				goto	wieg_endAL				; oli + front
				bcf		lastinputs,.6;7			; updateeri sisendite seisu
				btfsc	wiegandAto				; kui timeout siis on pakett juba käes ja loeme edasi vaid siis kui master on data ära lugenud
				goto	Andur_end				; minema siit !
				call	wieg_prepA				; + front, käivita paketi lõpu taimer
				call	rotL					; nihuta biti 0 väärtus registrisse
				incf	WAbitcount,F
				goto	Andur_end				; aitab !
Wiegand_Bhigh:
				banksel	PIE1
				bcf		PIE1,RC1IE
				banksel	.0
				btfsc	dataport,.5				; loeme vaid - fronti
				goto	wieg_endBH				; oli + front
				bcf		lastinputs,.5			; updateeri sisendite seisu
				btfsc	wiegandBto				; kui timeout siis on pakett juba käes ja loeme edasi vaid siis kui master on data ära lugenud
				goto	Andur_end				; minema siit !
				call	wieg_prepB				; + front, käivita paketi lõpu taimer
				call	rotH					; nihuta biti 1 väärtus registrisse
				incf	WBbitcount,F
				goto	Andur_end				; aitab !

Wiegand_Blow:
				banksel	PIE1
				bcf		PIE1,RC1IE
				banksel	.0
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
				LFSR	.0,Register11+.7			; jooxva biti salvestamiseks vajaliku registri aadress
				return
wieg_prepB:		movlw	wiegandtime
				movwf	wiegandBtimer
				bsf		wiegandBtmron			; taimer paketi kestust valvama
				LFSR	.0,Register15+.7			; jooxva biti salvestamiseks vajaliku registri aadress
				return
;===============================================================================
rotH:			bsf		CARRY
				goto	rot
rotL:			bcf		CARRY
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
				bcf		PIR1,RC1IF				; katkestuse nõue maha
				banksel	RCSTA1
				movf	RCSTA1,W				; oli viga?
				banksel	.0
				andlw	.6						; Viga vastuvõtul? Maskeeri tarbetud staatuse bitid
				btfss	ZERO
				goto	reset_ser2				; oli, alusta uuesti
modbus_rcv:		movf	bytecnt,W				; kontrolli puhvri piire
				sublw	RSBufLen-.1
				btfss	CARRY
				goto	reset_ser2				; liiga pikk mess või miskit sassis, reset!
				bsf		SerialTimerOn	
				banksel	RCREG1
				movf	RCREG1,W
				banksel .0
				movwf	Char
				LFSR	.0,Puhver				; arvutame baidi salvestamise koha vastuvõtupuhvris
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
				movf	bytecnt,W				; äkki oli teine (käsk) ?
				sublw	.2
				btfss	ZERO
				goto	modb_r1					; ei, päris mitmes oli...
				movf	Char,W
				sublw	modbus_cnf
				btfss	ZERO
				goto	modb_r00				; ei, äkki wr_multi ?
				movlw	.12;7
				movwf	countL
				goto	RREnd1					; jääb kuuldele...
modb_r00:		movf	Char,W
				sublw	modbus_wrmulti
				btfss	ZERO
				goto	modb_r1					; ei, siis ei näpi
				movlw	.7
				movwf	countL
				goto	RREnd1					; jääb kuuldele...


modb_r0:
modb_r12:
modb_r1:
modb_r14:
modb_r15:
modb_r16:
modb_r17:		movf	bytecnt,W
				subwf	countL,W				; saadetis käes (countL-s oodatav baitide arv)?
				btfss	ZERO
				goto	RREnd1					; eip !
				movff	Puhver+.1,WREG
				sublw	modbus_wrmulti			; kas oli käsk 0x10 (kirjuta mitu reg. korraga)?
				btfss	ZERO
				goto	modb_r2					; ei, pakett käes, kontrolli summat
				btfsc	cmd10
				goto	modb_r2
				movff	Puhver+.6,WREG			; jah, loeme saadetavate baitide arvu ja ootame nende saabumist
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
				LFSR	.0,Puhver				; kontrollime summat

				bcf		Dir						; lisa Dir signaal nulli
				bcf		PIR3,CCP2IF
				bsf		PIE3,CCP2IE				; ja lubame jälle sellise tekitamise

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
				bsf		cmd_ok					; aga märgi ära, et pakett oli ok
				bcf		SerialTimerOn			; taimer seisma
				banksel	PIE1
				bcf		PIE1,RC1IE				; enne uut käsku vastuvõtu ei võta kuni senine täidetud
				banksel	.0
				return
;===============================================
; ********* käskude täitmine *******************
;===============================================
command:		LFSR	.0,Puhver				; pakett OK, täidame käsu !
				movff	Register274+.1,WREG		; 1. bait on slave aadress. Kas jutt mulle ?
				subwf	INDF0,W
				btfsc	ZERO
				goto	rcv_1					; jutt minule
				bcf		broadcast
				movf	INDF0,W					; kas broadcast (adr. 0x00) ?
				addlw	.0
				btfss	ZERO
				goto	reset_ser1				; viga
				bsf		broadcast				; edaspidi kuulame broadcast-käske ka kuid neile ei vasta !
;				call	reload_side				; oli midagi muud, teeme ikkagi sidetaimeri reload aga ei vasta;
;				goto	reset_ser1				; 
;----------- CHG side reload iga baidi kuulmisel ------------------
rcv_1:			call	reload_side				; sidetaimeri reload
;----------- CHG side reload iga baidi kuulmisel ------------------
				incf	FSR0L,F
				movf	INDF0,W					; kas oli käsk RD holding register (0x03) ?
				sublw	modbus_rd
				btfsc	ZERO
				goto	modb_read				; jah
				movf	INDF0,W					; kas oli käsk WR holding register (0x06) ?
				sublw	modbus_wr
				btfsc	ZERO
				goto	modb_write				; jah
				movf	INDF0,W					; kas oli käsk WR conf register (0x45) ?
				sublw	modbus_cnf
				btfsc	ZERO
				goto	modb_conf				; jah
				movf	INDF0,W					; kas oli käsk WR multiple registers (0x10) ?
				sublw	modbus_wrmulti
				btfsc	ZERO
				goto	modb_multikulti			; jah
				goto	valekask				; teavita et oli vale käsk
;===============================================
; sidetaimeri reload
;===============================================
reload_side:	bsf		sidetmron
				movlw	sidetime
				movwf	sidetaimer
				movff	Register276,WREG		; taasta side kadumise viiteaeg
				movwf	reset1dlytmrH
				movff	Register276+.1,WREG	
				movwf	reset1dlytmrL
				movff	Register277,WREG		; taasta pulsi kestus reseti 1 generaatorile
				movwf	reset1pulsetmrH
				movff	Register277+.1,WREG	
				movwf	reset1pulsetmrL
				bsf		n_reset1				; inversioon
				bcf		reset1pulseon			; reseti 1 pulsi generaator OHV (igaks juhux)
rls1:;			bsf		pank1
				movff	Register278,WREG		; taasta side kadumise viiteaeg
				movwf	reset2dlytmrH
				movff	Register278+.1,WREG	
				movwf	reset2dlytmrL
				movff	Register279,WREG		; taasta pulsi kestus reseti 2 generaatorile
				movwf	reset2pulsetmrH
				movff	Register279+.1,WREG	
				movwf	reset2pulsetmrL
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
				goto	valedata				; ei ole, nii ütlegi

mbr0:			call	ch_algadr				; kas loetava algusaadress on piirides ?
				btfsc	CARRY
				goto	valedata				; ei ole, nii ütlegi
				btfsc	broadcast				; braodcast'i puhul ei vasta !
				goto	reset_ser				; muide, broadcast'iga lugeda olex eriti totter aga olgu piirang ikkagi - lolle leidub...

				bsf		clrsticky				; oletame, et lubataxe sticky't maha võtta
				btfss	reg0					; kas vaid 1 register & reg0 ? siis clrsticky =0 !!!
				goto	mbr00					; ei
				movff	n_regL,WREG				; loetavate registrite arv (LOW)
				sublw	.1
				btfss	ZERO
				goto	mbr00
				bcf		clrsticky				; jah, siis oli vaid reg. 0 lugemine => ei luba sticky't maha võtta

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
				btfsc	counters
				LFSR	.0,Loendi9
				movff	m_radrL,serpartmp
	bcf	CARRY
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
				btfss	dallas					; kas dallase asjad või loendid ?
				goto	modb_read_loop
				movlw	.164+.32;5
				addwf	FSR0L,F
				btfsc	CARRY
				incf	FSR0H,F
modb_read_loop:	movf	POSTINC0,W
				movwf	POSTINC1
				call	mb_crc16
				incf	bytecnt,F				; loendame baite
				decfsz	countL
				goto	modb_read_loop
				btfss	reg10					; kui loeti registrit 10, võib nüüd wiegandi bitiloendid nullida
				goto	modb_read_end
				btfss	wiegandAto				; wiegand A tulemus käes ?
				goto	modb_r_1				; eip
				clrf	WAbitcount				; jah - nulli A-wiegandi bitiloendid
				bcf		wiegandAto				; nüüd lubame uuesti A-wiegandi lugemise
				clrf	Register10+.0
				clrf	Register11+.0
				clrf	Register11+.1
				clrf	Register12+.0
				clrf	Register12+.1
				clrf	Register13+.0
				clrf	Register13+.1
				clrf	Register14+.0
				clrf	Register14+.1
modb_r_1:		btfss	wiegandBto				; wiegand B tulemus käes ?
				goto	modb_read_end			; eip
				clrf	WBbitcount				; jah - nulli B-wiegandi bitiloendid
				bcf		wiegandBto				; nüüd lubame uuesti B-wiegandi lugemise
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
; kirjuta ühte registrit
;===============================================
modb_write:		bsf		write
				call	validate				; kas aadress ja loetav pikkus piirides ?
				btfsc	CARRY
				goto	valedata				; ei ole, nii ütlegi

				movff	m_radrH,WREG			; kas register 258,259 ?
				sublw	0x01
				btfss	ZERO
				goto	modb_write00
				movff	m_radrL,WREG		
				sublw	0x02
				btfsc	ZERO
				goto	m_wr_chk				; on 258, seda ei luba kirjutada kui ta sisu !=0
 				movff	m_radrL,WREG		
				sublw	0x03
				btfsc	ZERO
				goto	m_wr_chk1				; on 259, seda ei luba kirjutada kui ta sisu !=0
				goto	modb_write00			; on mingi muu register, vaatab edasi	
m_wr_chk:		movff	Register258+.0,WREG		; kas R258 sisu =0 ?
				addlw	.0
				btfss	ZERO
				goto	valedata				; ei, siis ei luba kirjutada !
				movff	Register258+.1,WREG
				addlw	.0
				btfss	ZERO
				goto	valedata				; ei, siis ei luba kirjutada !
				goto	modb_write00			; Reg. 258=0 => võib kirjutada	
m_wr_chk1:		movff	Register259+.0,WREG		; kas R259 sisu =0 ?
				addlw	.0
				btfss	ZERO
				goto	valedata				; ei, siis ei luba kirjutada !
				movff	Register259+.1,WREG
				addlw	.0
				btfss	ZERO
				goto	valedata				; ei, siis ei luba kirjutada !
				goto	modb_write00			; Reg. 259=0 => võib kirjutada	

modb_write00:	call	ch_algadr				; kas kirjutatava algusaadress on piirides ja kas lubatud kirjutada ?
				btfsc	CARRY
				goto	valedata				; ei ole, nii ütlegi
				btfsc	readonly				; kirjutamine lubatud ?
				goto	valedata				; ei ole, nii ütlegi
				btfsc	special0				; olid erikäsud ?
				goto	modb_write2				; jah, siis vaid vastame
				btfsc	special1				; olid erikäsud ?
				goto	modb_write2				; jah, siis vaid vastame
				btfsc	pwm						; loeti pwmi registreid ?
				goto	modb_write0				; jah, FSR0 on juba laetud
				LFSR	.0,Register0
				btfsc	counters
				LFSR	.0,Loendi9
				movff	m_radrL,serpartmp
				rlcf	serpartmp,W				; liidame puhvri alguse (aadress *2)
				addwf	FSR0L,F
				btfsc	CARRY	
				incf	FSR0H,F					; loetava registri aadress olemas !
				btfss	dallas					; kas loendid ?
				goto	modb_write0				; eip !
				movlw	.164+.32;5					; jah, need nati kaugemal
				addwf	FSR0L,F
				btfsc	CARRY
				incf	FSR0H,F

modb_write0:	movff	n_regH,WREG				; kirjutab data (HIGH)
				movwf	POSTINC0
				btfsc	sync					; PWMi kirjutame ka tööregistrisse
				movwf	POSTINC1
				movff	n_regL,WREG				; kirjutab data (LOW)
				movwf	INDF0
				btfsc	sync
				movwf	POSTINC1
				btfss	reg0					; oli register 0 ?
				goto	modb_write1				; eip !
				movff	Register0,WREG			; jah, dubleerime kohe porti ka
				movwf	DoutPort				; digiväljundid ja PU-d
				movff	Register0+.1,Registertemp7	
;----
				andlw	0x0F
				movwf	PORTA					; ja bitikaupa miskipärast ei lähe !?
				btfsc	Registertemp7,.4		; kombineerime baidid ANA-pordi juhtimisex	
				bsf		PORTA,.5
;				addlw	0x20					; analoog või sisendi puhul kirjutamine nagunii ei mõju
				bcf		CARRY
				rrcf	Registertemp7,F
				swapf	Registertemp7,W
				andlw	0x07
				movwf	PORTE

modb_write1:	bcf		reg0				
				btfss	res_pwm					; kirjutati reg.149 või 150-sse ?
				goto	modb_write2
				call	reset_pwm				; jep! reseti PWM !
				call	reset_per				; ja tee periood nullix ja väljundid kah !

				movff	Register150+.0,WREG		; kas periood < 2 ?
				andlw	0x0F
				btfss	ZERO
				goto	modb_write2
				movff	Register150+.1,WREG
				cpfsgt	0x02
				goto	modb_write2
;				LFSR	.0,pwm0work				; on 0, teeme kõik korduvd pulsid ühekordseks
				LFSR	.1,pwm0set
				movlw	.16
				movwf	_1mscount
set_zer:;		bcf		POSTINC0,.7
		;		movf	POSTINC0,W
				bcf		POSTINC1,.7
				movf	POSTINC1,W
				decfsz	_1mscount
				goto	set_zer
				movlw	.4
				movwf	_1mscount

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
modb_multikulti:bcf		writedsID				; oletame, et ei ole tegu Dallase ID-dega
				movff	m_radrL,WREG			; multi-kultit lubame vaid PWMi regitritele			
				sublw	.100-.1					; kas <100 ?
				btfsc	CARRY
				goto	valedata				; oli liiga madal aadress
				movff	m_radrL,WREG				
				sublw	.115					; kas >115 ?
				btfss	CARRY
				goto	mk_1					; oli liiga kõrge aadress, äkki Dallase kividele või loendid ?
				goto	mk_pwm					; olid PWMi asjad


mk_1:			movff	m_radrH,WREG			; kas bootloaderile ?
				sublw	HIGH(.998)
				btfsc	ZERO
				goto	mk_1_2					; jep, asi läheb karmix
; ei, kas kirjutame Dallase asju (ID-d)?
				movff	m_radrH,XH				; adr >685 ?
				movff	m_radrL,XL
				movlw	HIGH(.685)
				movwf	YH
				movlw	LOW(.685)
				movwf	YL
				call	compare16_16
				btfss	CARRY			
				goto	validate2
				movlw	HIGH(.649)				; adr <650 ?
				movwf	YH
				movlw	LOW(.649)
				movwf	YL
				call	compare16_16
				btfsc	CARRY
				goto	mk_1_1					; siis pole DS1820 ID-d
; kalkuleeri DS1820 ID-registri aadress
				movff	m_radrL,serpartmp
				movlw	0x8A					; teisenda anduri aadress: high = 0, low -0x8A
				subwf	serpartmp,F;W
				LFSR	.0,Dallas1wa			; kirjutame DS1820 ID-desse
				bcf		CARRY
				rlcf	serpartmp,W				; liidame puhvri alguse (aadress *2)
				addwf	FSR0L,F
				btfsc	CARRY	
				incf	FSR0H,F					; loetava registri aadress olemas !
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
				movlw	HIGH(LoendiUP)
				cpfsgt	mb_temp1
				goto	mkd1
				goto	valedata
mkd1:			cpfseq	mb_temp1
				goto	mkd2					; on OK !
				movlw	LOW(LoendiUP)
				cpfsgt	mb_temp2
				goto	mkd2					; on OK !
				goto	valedata
mkd2:			movf	n_regH,W				; kirjutavate registrite arv (HIGH)
				movwf	countH
				movff	n_regL,countL
				rlcf	countL,F				; kirjutavate registrite arv (LOW)
				movf	countH,W				; kui kirjutatava pikkus =0, siis per...
				addlw	.0
				sublw	.0
				btfss	ZERO
				goto	mkl3a					; kirjuta !
				movf	countL,W				
				addlw	.0
				sublw	.0
				btfsc	ZERO
				goto	valedata
				goto	mkl3a					; kirjuta !				
; aadressi kontroll jätkub, kas DS2438 või mis ta raisa nimi oligi...			
mk_1_1:			movff	m_radrH,XH				; adr >785 ?
				movff	m_radrL,XL
				movlw	HIGH(.785)
				movwf	YH
				movlw	LOW(.785)
				movwf	YL
				call	compare16_16
				btfss	CARRY			
				goto	validate2
				movlw	HIGH(.749)				; adr <750 ?
				movwf	YH
				movlw	LOW(.749)
				movwf	YL
				call	compare16_16
				btfsc	CARRY
				goto	mk_loendid				; siis pole DS2843 ID-d. Ehk loendid ?
; kalkuleeri DS2438 ID-registri aadress
				movff	m_radrL,serpartmp
				movlw	0x8A					; teisenda anduri aadress: high = 0, low -0x8A
				subwf	serpartmp,F;W
				LFSR	.0,DS24381wa			; kirjutame DS2843 ID-desse
				bcf		CARRY
				rlcf	serpartmp,W				; liidame puhvri alguse (aadress *2)
				addwf	FSR0L,F
				btfsc	CARRY	
				incf	FSR0H,F					; loetava registri aadress olemas !
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
				movlw	HIGH(ch1meascnt)
				cpfsgt	mb_temp1
				goto	mkdd1
				goto	valedata
mkdd1:			cpfseq	mb_temp1
				goto	mkdd2					; on OK !
				movlw	LOW(ch1meascnt)
				cpfsgt	mb_temp2
				goto	mkdd2					; on OK !
				goto	valedata
mkdd2:			movf	n_regH,W				; kirjutavate registrite arv (HIGH)
				movwf	countH
				movff	n_regL,countL
				rlcf	countL,F				; kirjutavate registrite arv (LOW)
				movf	countH,W				; kui kirjutatava pikkus =0, siis per...
				addlw	.0
				sublw	.0
				btfss	ZERO
				goto	mkl3a					; kirjuta !
				movf	countL,W				
				addlw	.0
				sublw	.0
				btfsc	ZERO
				goto	valedata
				goto	mkl3a					; kirjuta !
; bootloaderile kirjutamine !
mk_1_2:			movff	m_radrL,WREG			
				sublw	LOW(.998)			
				btfss	ZERO
				goto	mk_loendid				
				movff	n_regH,WREG				; jah, saadetakse slave ID ja blokkide arv: registrite arv peab olema 2
				addlw	.0
				btfss	ZERO
				goto	valedata				; -> per...
				movff	n_regL,WREG				
				sublw	.2
				btfss	ZERO
				goto	valedata				
				LFSR	.2,Puhver+.6			; daata tuleb siit 
				movff	Register258+.1,WREG		; peab olema selle slave ID
				subwf	POSTINC2,W
				btfss	ZERO
				goto	valedata				
				movff	Register259+.1,WREG		; õige ?
				subwf	POSTINC2,W
				btfss	ZERO
				goto	valedata				
				movff	POSTINC2,BlBlockCount	; 64-baidiste blokkide arv

				call	paketialgus				; kirjuta paketi algus (ADR, CMND) puhvrisse	
				movff	m_radrH,WREG			; start aadress (998)
				movwf	POSTINC1
				call	mb_crc16
				incf	bytecnt,F				; loendame baite
				movff	m_radrL,WREG
				movwf	POSTINC1
				call	mb_crc16
				incf	bytecnt,F				; loendame baite
				movlw	.0						; 64-baidiste blokkide arv
				movwf	POSTINC1
				call	mb_crc16
				incf	bytecnt,F				; loendame baite
				movff	BlBlockCount,WREG		
				movwf	POSTINC1
				call	mb_crc16
				incf	bytecnt,F				; loendame baite
				movf	_RS485chk,W				; ja summa
				movwf	POSTINC1
				incf	bytecnt,F				; loendame baite
				movf	_RS485chkH,W	
				movwf	POSTINC1
				incf	bytecnt,F				; loendame baite
;				call	dly						; 3,98 ms viidet
				LFSR	.1,Puhver				; hakkab saatma
				bsf		Dir
mk_2:			movf	POSTINC1,W
				call	SendCHAR
				decfsz	bytecnt
				goto	mk_2
				bcf		Dir
; nüüd tuleb end bootloadimiseks valmis seada...

				goto	reset_ser


				
mk_loendid:		movff	m_radrH,WREG
				sublw	HIGH(.384)
				btfss	ZERO
				goto	valedata				; oli vale aadress
				movff	m_radrL,WREG			; multi-kultit lubame nüüd ka loendite regitritele			
				sublw	LOW(.384-.1)			; kas <384 ?
				btfsc	CARRY
				goto	valedata				; oli liiga madal aadress
				movff	m_radrL,WREG				
				sublw	LOW(.415)				; kas >415 ?
				btfss	CARRY
				goto	valedata				; oli liiga kõrge aadress
				movff	m_radrL,serpartmp
				movlw	0x80					; loendid
				subwf	serpartmp,W
				movff	WREG,m_radrL
				movff	WREG,serpartmp
				LFSR	.0,Loendi9				; kirjutame loenditesse
				bcf		CARRY
				rlcf	serpartmp,W				; liidame puhvri alguse (aadress *2)
				addwf	FSR0L,F
				btfsc	CARRY	
				incf	FSR0H,F					; loetava registri aadress olemas !
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
				movlw	HIGH(Dallas1)
				cpfsgt	mb_temp1
				goto	mkl1
				goto	valedata
mkl1:			cpfseq	mb_temp1
				goto	mkl2					; on OK !
				movlw	LOW(Dallas1)
				cpfsgt	mb_temp2
				goto	mkl2					; on OK !
				goto	valedata
mkl2:			movf	n_regH,W				; kirjutavate registrite arv (HIGH)
				movwf	countH
				movff	n_regL,countL
				rlcf	countL,F				; kirjutavate registrite arv (LOW)
				movf	countH,W				; kui kirjutatava pikkus =0, siis per...
				addlw	.0
				sublw	.0
				btfss	ZERO
				goto	mkl3
				movf	countL,W				
				addlw	.0
				sublw	.0
				btfsc	ZERO
				goto	valedata
mkl3a:			bsf		writedsID				; märgime, et dallase ID-d tuleb salvestada
mkl3:			LFSR	.2,Puhver+.7			; daata tuleb siit (pos. 7 sest eeline on BAITIDE arv !
mkloop:			movff	POSTINC2,POSTINC0
				decfsz	countL
				goto	mkloop
				goto	modb_mk_end
;**** pwmi kirjutamine ****
mk_pwm:			movff	m_radrL,serpartmp		; oli PWMi register
				movlw	.100
				subwf	serpartmp,W
				movff	WREG,serpartmp
				LFSR	.0,pwm0set				; kirjutame nii set kui ka tööregistrisse !
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
				incf	FSR1H,F					; tööregistri aadress samuti olemas !
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
modb_mk:		LFSR	.2,Puhver+.6			; daata tuleb siit 
modb_mkloop:	movff	INDF2,POSTINC1
				movff	POSTINC2,POSTINC0
				decfsz	countL
				goto	modb_mkloop
modb_mk_end:	btfsc	broadcast				; broadcast'i puhul ei vasta !
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
				movff	Register258+.1,WREG		; õige ?
				subwf	INDF0,W
				btfss	ZERO
				goto	reset_ser				; jama !
				incf	FSR0L,F					; viita seerianumbrile (LOW)
				movff	Register259+.1,WREG		; õige ?
				subwf	INDF0,W
				btfss	ZERO
				goto	reset_ser				; jama !
				bsf		writeit					; paneme peale vastamist parameetrid kirja ka !
				incf	FSR0L,F					; üle ser. nr. korduse
				incf	FSR0L,F					; üle ser. nr. korduse
				incf	FSR0L,F					; üle ser. nr. korduse
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
				movf	INDF0,W					; AIO suund
				movff	WREG,Register275+.1
; >>>CHG<<<
;				movlw	0x00					; nüüd AIO ana/digi omadus, siin ei muuda !
;				movff	WREG,Register275
; >>>CHG<<<
				btfsc	broadcast				; braodcast'i puhul ei vasta !
				goto	reset_ser
; Vastus: ADR, CMD, ID1, ID2, ID1, ID2, CRCH,CRCL
				call	paketialgus				; kirjuta paketi algus (ADR, CMND) puhvrisse	
				movff	serpartmp,Register274+.1; jõustame uue aadressi
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
;				call	dly						; 3,98 ms viidet
				LFSR	.1,Puhver				; hakkab saatma
				bsf		Dir
send1:			movf	POSTINC1,W
				call	SendCHAR
				decfsz	bytecnt
				goto	send1
				bcf		Dir
				btfss	writedsID				; saadeti Dallase kivi ID ja see tuleb nüüd salvestada ?
				goto	send1_1					; eip
				bcf		INTCON,GIE
				call	save_dallas_adr			; jepp!
				bcf		writedsID
				bsf		INTCON,GIE
send1_1:		btfss	writeit					; oli konfidaata ja see tuleks EEPROMi kirjutada?
				goto	reset_ser
				bcf		INTCON,GIE
				bcf		INTCON,PEIE
				call	Save_Setup				; jah
				bsf		INTCON,GIE
				bsf		INTCON,PEIE

send2:			bcf		writeit					; kirjutatud !
				goto	reset_ser;1				; lõpetame jama ää...
;===============================================================================
paketialgus:	LFSR	.1,Puhver				; formeerime vastuse saatepuhvrisse
				movlw	0xFF					
				movwf	_RS485chkH				; valmistume kontrollsumma arvutamiseks	
				movwf	_RS485chk
				clrf	bytecnt
;--- CHG1 ---
				movf	INDF1,W					; loe oma aadress saatest (sest see pidi ju selline olema !)
				btfsc	broadcast				; broadcast'i puhul võtame oma aadressi EEPROMist
;--- CHG1 ---
				movff	Register274+.1,WREG		; oma aadress
				movwf	POSTINC1
				call	mb_crc16
				incf	bytecnt,F				; loendame baite
				movf	POSTINC1,W				; käsk juba kirjas, arvutame ta CRC sisse ja loendame baite kah
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
				movlw	HIGH(.383)				; adr <384 ?
				movwf	YH
				movlw	LOW(.383)
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
				movlw	HIGH(.610)				; max=610 -> DS18B20 näidud
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
validate9:		movff	m_radrH,XH				; adr >699 ?
				movff	m_radrL,XL
				movlw	HIGH(.699)
				movwf	YH
				movlw	LOW(.699)
				movwf	YL
				call	compare16_16
				btfss	CARRY			
				goto	validate9a				; jah
				movff	m_radrH,WREG			; adr =699 ?
				sublw	HIGH(.699)
				btfss	ZERO
				goto	validate_bad			; siis error !			
				movff	m_radrL,WREG
				sublw	LOW(.699)
				btfss	ZERO
				goto	validate_bad			; siis error !			
				goto	validate_ok				; korras !

validate9a:		movff	m_radrH,XH				; adr >735 ?
				movff	m_radrL,XL
				movlw	HIGH(.735)
				movwf	YH
				movlw	LOW(.735)
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
				movlw	HIGH(.737)				; max=733 -> DS2438 näidud
				movwf	XH
				movlw	LOW(.737)
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
validate_end:	movff	Puhver+.1,WREG			; registrite arv anti vaid käskude read (0x03) ja wr_multi (0x10) puhul
				sublw	modbus_wr
				btfsc	ZERO
				goto	validate_ok				; käsk WRITE, siis reg. arvu ei anta, on OK
				movff	Puhver+.1,WREG			
				sublw	modbus_cnf
				btfsc	ZERO
				goto	validate_ok				; käsk CONF, siis reg. arvu ei anta, on OK
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
				bcf		counters				; ei olnud loendid
				bcf		special0
				bcf		special1
				movff	m_radrL,WREG
				movff	WREG,Puhver+RSBufLen-.1
				movff	m_radrH,WREG			; kas register 0 ?
				movff	WREG,Puhver+RSBufLen-.2
				addlw	.0
				btfss	ZERO
				goto	ch_alg0
				movff	m_radrL,WREG
				addlw	.0
				btfss	ZERO
				goto	ch_alg0;1
				bsf		reg0					; et alustati registrist 0
				goto	chk_algok				; on r/w register 0, kõik OK
ch_alg0:		movff	m_radrL,WREG			; aadressi kontroll jätkub
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
				movff	m_radrL,WREG			; aadressi kontroll jätkub
				sublw	0x96					; kas pwmi registrid 100...115,150 ?
				btfsc	ZERO
				goto	chk_alg12				; oli register 150
				movff	m_radrL,WREG
				sublw	0x97					; 
				btfsc	ZERO
				goto	chk_alg12a				; oli register 151
				btfss	CARRY
				goto	chk_algbad				; oli liiga kõrge aadress
				movff	m_radrL,WREG				
				sublw	.100-.1					; kas <100 ?
				btfsc	CARRY
				goto	chk_algbad				; oli liiga madal aadress
				movff	m_radrL,WREG				
				sublw	.115					; kas >115 ?
				btfss	CARRY
				goto	chk_algbad				; oli liiga kõrge aadress
				goto	chk_alg11				; pwmi register oli

chk_alg1:		bsf		readonly				; on vaid loetav register
chk_alg1a:		movlw	.0
				movff	WREG,m_radrH
				goto	chk_algok				; OK
chk_alg3:		movff	m_radrH,WREG
				sublw	.2
				btfsc	ZERO
				goto	chk_alg8				; tegu Dallase juraga
				movff	m_radrH,WREG
				sublw	.3
				btfsc	ZERO
				goto	chk_alg8				; tegu Dallase juraga aga ilmselt DS2438
;**** uptime loendamine ****
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
;				bsf		dallas					; ei ole dallase asi aga on loendi mis asub samuti taamal...
				movff	m_radrL,WREG			; on loendid, algavad 0x180-st
				sublw	0x80-.1						
				btfss	CARRY
				goto	chk_alg7;6				; Loendid on r/w registrid

chk_alg3b:		bcf		dallas					; ei ole dallase asi ei ole loendi kah, parandame
				movff	m_radrL,WREG			; reg. >= 256, kas 0x0100, 0x0102 või 0x0103 ?
				sublw	0x00
				btfss	ZERO
				goto	chk_alg3a
				movlw	.19						; 256dec -> jrk.nr. 19dec , r-only, dev. type
				movff	WREG,m_radrL
				goto	chk_alg1
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
				movlw	.21						; 258 -> 21, nyyd r/w, serial High
				movff	WREG,m_radrL
				goto	chk_alg1a
chk_alg4:		movff	m_radrL,WREG				
				sublw	0x03
				btfss	ZERO
				goto	chk_alg5
				movlw	.22						; 259 -> 22, nyyd r/w, serial Low
				movff	WREG,m_radrL
				goto	chk_alg1a
chk_alg5:		movff	m_radrL,WREG			; adr 261...270 välja !
				sublw	LOW(.270-.1)			; kas < 270 ?
				btfsc	CARRY
				goto	chk_algbad				; siis per...
				movff	m_radrL,WREG			; kas > 279;7 ?
				sublw	LOW(.280-.1)
				btfss	CARRY
				goto	chk_algbad				; siis per...
;_____
				movff	Puhver+.1,WREG			; kas oli lugemine ?
				sublw	modbus_rd
				btfsc	ZERO
				goto	chk_alg5a				; käsk READ,ei tee mingit resetti ega ka käse EEPROMi kirjutada  !!!
;_____
				bsf		writeit					; oli konfidaata.kirjutada EEPROMi !
				movff	m_radrL,WREG			; kas = 273 ?
				sublw	LOW(.273)
				btfss	ZERO
				goto	chk_alg5a				; eip !
; 29.1.2014.: ei tee enam reset1-pulssi !
;				bsf		reset1ena				; jah, siis teeme kohe ka reset 1-e
;				movlw	.0
;				movwf	reset1dlytmr
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
				bsf		clrsticky				; ja järelikult võib sticky biti ka maha võtta peale lugemist
				movff	m_radrL,WREG
				sublw	.10						; kas oli reg. 10 lugemine ?
				btfsc	ZERO
				bsf		reg10
				goto	chk_alg1				; jah, need kõik read-only !
chk_alg7:		movff	m_radrL,serpartmp
				movlw	0x80;.144;112					; loendid
;				addwf	serpartmp,W
				subwf	serpartmp,W
				movff	WREG,m_radrL
				movlw	.0
				movff	WREG,m_radrH
				bsf		counters
				goto	chk_alg6				; need on kõik r/w registrid

chk_alg8:		movff	m_radrH,WREG			; teha reset (äksessiti reg. 999) ?
				sublw 	HIGH(.999)				
				btfss	ZERO
				goto	chk_alg8a				; ei
				movff	m_radrL,WREG
				sublw 	LOW(.999)				; teha reset (äksessiti reg. 999) ?
				btfss	ZERO
				goto	chk_alg8a				; ei
; lubame reseti vaid wr käsu puhul kui kirjutati data 0xDEAD
				movff	Puhver+.1,WREG
				sublw	0x06
				btfss	ZERO
				goto	chk_alg8_1
				movff	n_regH,WREG
				sublw	0xDE
				btfss	ZERO
				goto	chk_alg8_1
				movff	n_regL,WREG
				sublw	0xAD
				btfss	ZERO
				goto	chk_alg8_1
				bcf		INTCON,GIE
				bcf		INTCON,PEIE			
				reset							; jah, laseme reseti teha
chk_alg8_1:		movff	Puhver+.1,WREG			; kui dc1a ja 699=0, tee discovery
				sublw	0x06
				btfss	ZERO
				goto	chk_alg8_2
				movff	n_regH,WREG
				sublw	0xDC
				btfss	ZERO
				goto	chk_alg8_2
				movff	n_regL,WREG
				sublw	0x1A
				btfss	ZERO
				goto	chk_alg8_2
				movff	Register699+.1,W		; käsk õige, kas 699=0 ?
				addlw	.0
				btfss	ZERO
				goto	chk_algbad				; võlssivad raiped
				bsf		special0				; märgime, et oli erikäsk 0
				goto	chk_algok
;2. kirjutades 999 sisse koodi feed võiks käivituda reset1 hoolimata 276 sisust ja kesta vastavalt 277 poolt määratule.
chk_alg8_2:		movff	Puhver+.1,WREG			; kui feed, tee reset1
				sublw	0x06
				btfss	ZERO
				goto	chk_alg8_3
				movff	n_regH,WREG
				sublw	0xFE
				btfss	ZERO
				goto	chk_alg8_3
				movff	n_regL,WREG
				sublw	0xED
				btfss	ZERO
				goto	chk_alg8_3
				bsf		special1				; märgime, et oli erikäsk 1
				goto	chk_algok
chk_alg8_3:		goto	chk_algbad

chk_alg8a:		movff	m_radrH,WREG			; = 699 ?
				sublw	.2
				btfss	ZERO
				goto	chk_alg8aa
				movff	m_radrL,WREG
				sublw	0xBB	
				btfss	ZERO
				goto	chk_alg8aa				; eip !
				LFSR	.0,Register699
				bsf		pwm						; märgime, et FSR0 on juba laetud
				bcf		loendi
				bcf		dallas
				bcf		readonly
				bsf		writeit					; laseme kirjutada EEPROMi !
				goto	chk_algok

chk_alg8aa:		movff	m_radrH,WREG			; > 700 ? Siis DS2438 kivi
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
				goto	chk_alg13				; jah, DS2438 kivi jutud käivad !

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
				bsf		dallas
				goto	chk_alg1				; jah, need on r/0 registrid !
				
chk_alg9:		movff	m_radrL,serpartmp
				movlw	.81						; teisenda anduri ID aadress: high = 0, low -81
				subwf	serpartmp,W
				movff	WREG,m_radrL
				movlw	.0
				movff	WREG,m_radrH
				bsf		dallas
;				goto	chk_alg1				; jah, need on r/0 registrid !
				goto	chk_algok				; nüüd lubatud kirjutada !
; pwm-i registrid
chk_alg11:		movff	m_radrL,serpartmp
				movlw	.100
				subwf	serpartmp,W
				movff	WREG,m_radrL
				movff	m_radrL,serpartmp				
				LFSR	.0,pwm0set				; kirjutame nii set kui ka tööregistrisse !
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
				incf	FSR1H,F					; tööregistri aadress samuti olemas !
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
				bsf		readonly				; R151 on nüüd r/o !
				goto	chk_algok
chk_alg12b:		bsf		pwm
				bsf		res_pwm					; kirjutati reg-sse 149, tee PWMile reset
				LFSR	.0,Register149out+.0
				btfsc	write
				LFSR	.0,Register149in+.0
				goto	chk_algok

chk_alg13:		movff	m_radrH,WREG
				sublw	.3
				btfsc	ZERO
				goto	chk_alg14				; DS2438 kivi ID-de registrite pöördumine
				movff	m_radrL,WREG
				sublw	0xEE-.1					; > (LOW(750) -1) ?
				btfss	CARRY
				goto	chk_alg14				; DS2438 kivi ID-de registrite pöördumine
; DS2438 näidud
				LFSR	.0,DS2438_1				; näitude registrite baas
				movlw	.0
				movff	WREG,m_radrH
				movff	m_radrL,serpartmp
				movlw	LOW(.700)				; aadress-0xBC
				subwf	serpartmp,W	
				movff	WREG,m_radrL
				bcf		CARRY
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
;				goto	chk_alg1				; read-only !
; nüüd on ka ID kirjutamine lubatud !
chk_algok:		bcf		CARRY					; algusaadress oli lubatud piirides => Cy=0
				return
chk_algbad:		bsf		CARRY					; algusaadress oli üle piiri => Cy=1
				bcf		clrsticky	
				return
;===============================================================================
valekask:		movlw	IllFunc					; vale käsk, mine per...
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
				movf	POSTINC1,W				; bump'i pointerit
				movff	Puhver+.1,WREG
				call	mb_crc16
				incf	bytecnt,F				; loendame baite
				movf	POSTINC1,W				; vea kood siin juba kirjas, arvutame CRC-sse ja loendame ikkagi
				call	mb_crc16
				incf	bytecnt,F				; loendame baite
				goto	paketilopp				; summa puhvisse ja saada teele
;===============================================================================
reset_ser2:		goto	reset_ser
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
				bsf		PIE3,CCP2IE				; ja lubame jälle sellise tekiamise
				return
RREnd1:			movlw	serialtime				; relae ootetaimer
				movwf	serialtimer
				return
;===============================================================================
; ********************** funktsioonid ******************************************
;===============================================================================
inc_count:		bcf		CARRY
				movf	INDF0,W
				addlw	.1
				movwf	INDF0					; loendame 1 hoolega ära debouncetud pulsi juurde
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
; * T1 on süsteemi taimer intervalliga 10 ms.
T1int:			bcf     PIR1,TMR1IF    			; katkestuse nõue maha 
				movlw	T1resoL					; lae taimer 1 uuesti
				movwf	TMR1L
				movlw	T1resoH
				movwf	TMR1H
;********* Vahipeni ketti **************
				clrwdt                    		; WDT nullida 
;**** Wiegandi paketi lõpu taimerid ****
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
				btfsc	wiegandBtmron			; kui teise lugeja lugemine pooleli, ei lase veel sidet üles võtta
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
				btfsc	wiegandAtmron			; kui teise lugeja lugemine pooleli, ei lase veel sidet üles võtta
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
				btfss	sidetmron				; kui 30s jooksul meiega ei suheldud, võtab default sideparameetrid
				goto	T1int_0
				decfsz	sidetaimer
				goto	T1int_0
				call	setup_serial0			; võtab vaikimisi seriali seaded
				call	reset_ser1				; sidetaimeri reload
				bcf		sidetmron				; sidetaimer seisma
;---------- on seda vaja ? -------------
				bcf		PIR1,RC1IF
				banksel	PIE1
				bsf		PIE1,RC1IE				; lubame uue käsu vastuvõttu kui miskipärast oli keelatud
				banksel	.0
;---------- on seda vaja ? -------------
;**** Resettide taimerid ****
T1int_0:		call	decrtmrs
;**** sidepaketi taimer ****
T1int_1:		btfss	SerialTimerOn			; seriali taimer käib?
				goto	T1int_2
				decf	serialtimer,F			; aeg täis?
				movf	serialtimer,W
				addlw	.0
				btfss	ZERO
			goto	T1int_10; 2
				call	reset_ser
				bcf		PIR1,RC1IF				; katkestuse nõue maha
			goto	T1int_10
; *** loendite sisendite debounce ***
T1int_2:		btfss	sens1tmron				; loendi 1: debounceme ?
				goto	T1int_3					; eip
				decfsz	Loendi1tmr
				goto	T1int_3
				movlw	senstime				; on aeg !
				movwf	Loendi1tmr
				bcf		sens1tmron
				LFSR	.0,Loendi1+.3
;				call	inc_count				; tixub 1 pulsi 
T1int_3:		btfss	sens2tmron				; loendi 2
				goto	T1int_4					
				decfsz	Loendi2tmr
				goto	T1int_4
				movlw	senstime			
				movwf	Loendi2tmr
				bcf		sens2tmron
				LFSR	.0,Loendi2+.3
;				call	inc_count			
T1int_4:		btfss	sens3tmron				; loendi 3
				goto	T1int_5				
				decfsz	Loendi3tmr
				goto	T1int_5
				movlw	senstime				
				movwf	Loendi3tmr
				bcf		sens3tmron
				LFSR	.0,Loendi3+.3
;				call	inc_count				
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
				goto	T1int_10				
				decfsz	Loendi8tmr
				goto	T1int_10
				movlw	senstime				
				movwf	Loendi8tmr
				bcf		sens8tmron
				LFSR	.0,Loendi8+.3
				call	inc_count				
;
T1int_10:
;**** sisendite debounce ***** - > sisend 0 (DIN plokist)
Din_0:			movf	DinPort,W				; loeme digi-pordi seisu
				movwf	Registertemp5
;				xorwf	lastinputs2,W			; mis muutus ?
;				movwf	muutus
;;;;>>>>>
;				movf	Register271+.0,W		; lisa: XORime tulemuse sellega läbi
			comf	Register271+.0,W		; lisa: XORime tulemuse sellega läbi
				xorwf	Registertemp5,F
;
				movf	Registertemp5,W
				xorwf	lastinputs2,W			; mis muutus ?
				movwf	muutus1
;
				btfss	in_sticky
				goto	Din_0a
				btfsc	dinstuck0
				goto	Din_1
Din_0a:			btfsc	Din0					; sisend 0 madal ?
				goto	Din0high				; ei, kõrge, seedi seda
;			bcf		lastinputs2,.0			; oli vist langev front, updateerime sisendite eelmist seisu
				btfsc	in_debounce				; kas debounce't teeb ?
				goto	Din_0stk				; ei, arvestab kohe
				btfss	d0timeron				; jah, kas juba teame ?
				goto	Din0high1				; ei, käivita deb. timer kui vaja
				decfsz	din0tmr					; debouncetud?
				goto	Din_1					; ei veel, võta järgmine sisend
Din_0stk:		bsf		dinpress0				; jah, võtame arvesse
				bsf		dinstuck0				; blokeerime biti igal juhul
		bcf		lastinputs2,.0			; oli vist langev front, updateerime sisendite eelmist seisu
			btfss	muutus1,.0				; oli front või selline seis jätkub ?
			goto	Din_0setlow				; senine seis jätkub
			LFSR	.0,Loendi1+.3
			call	inc_count				; tixub 1 pulsi 
Din_0setlow:	bsf		Register1,.0
Din0_lstckl:	bcf		d0timeron
				goto	Din_1					; võta järgmine sisend
Din0high:bsf		lastinputs2,.0			; oli vist tõusev front, updateerime sisendite eelmist seisu
				btfss	dinpress0				; oli kõrge enne ?
				goto	Din_1					; ei, võta järgmine sisend
				btfsc	in_debounce				; kas debounce't teeb ?
				goto	Din_0stkh				; ei, arvestab kohe
				btfss	d0timeron				; lahti laskmise debounce käib?
				goto	Din0strt				; ei, paneme käima
				decfsz	din0tmr
				goto	Din_1					; võta järgmine sisend
				bcf		dinpress0				; loeme lahti lastuks
Din_0stkh:		bsf		dinstuck0				; blokeerime biti igal juhul
Din_0sethi:		bcf		Register1,.0
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
;			bcf		lastinputs2,.1			; oli vist langev front, updateerime sisendite eelmist seisu
				btfsc	in_debounce				; kas debounce't teeb ?
				goto	Din_1stk				; ei, arvestab kohe
				btfss	d1timeron				; jah, kas juba teame ?
				goto	Din1high1				; ei, käivita deb. timer kui vaja
				decfsz	din1tmr					; debouncetud?
				goto	Din_2					; ei veel, võta järgmine sisend
Din_1stk:		bsf		dinpress1				; jah, võtame arvesse
				bsf		dinstuck1				; blokeerime biti igal juhul
		bcf		lastinputs2,.1			; oli vist langev front, updateerime sisendite eelmist seisu
			btfss	muutus1,.1				; oli front või selline seis jätkub ?
			goto	Din_1setlow				; senine seis jätkub
			LFSR	.0,Loendi2+.3
			call	inc_count				; tixub 1 pulsi 
Din_1setlow:	bsf		Register1,.1
Din1_lstckl:	bcf		d1timeron
				goto	Din_2					; võta järgmine sisend
Din1high:bsf		lastinputs2,.1			; oli vist tõusev front, updateerime sisendite eelmist seisu
				btfss	dinpress1				; oli kõrge enne ?
				goto	Din_2					; ei, võta järgmine sisend
				btfsc	in_debounce				; kas debounce't teeb ?
				goto	Din_1stkh				; ei, arvestab kohe
				btfss	d1timeron				; lahti laskmise debounce käib?
				goto	Din1strt				; ei, paneme käima
				decfsz	din1tmr
				goto	Din_2					; võta järgmine sisend
				bcf		dinpress1				; loeme lahti lastuks
Din_1stkh:		bsf		dinstuck1				; blokeerime biti igal juhul
Din_1sethi:		bcf		Register1,.1
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
;			bcf		lastinputs2,.2			; oli vist langev front, updateerime sisendite eelmist seisu
				btfsc	in_debounce				; kas debounce't teeb ?
				goto	Din_2stk				; ei, arvestab kohe
				btfss	d2timeron				; jah, kas juba teame ?
				goto	Din2high1				; ei, käivita deb. timer kui vaja
				decfsz	din2tmr					; debouncetud?
				goto	Din_3					; ei veel, võta järgmine sisend
Din_2stk:		bsf		dinpress2				; jah, võtame arvesse
				bsf		dinstuck2				; blokeerime biti igal juhul
		bcf		lastinputs2,.2			; oli vist langev front, updateerime sisendite eelmist seisu
			btfss	muutus1,.2				; oli front või selline seis jätkub ?
			goto	Din_2setlow				; senine seis jätkub
			LFSR	.0,Loendi3+.3
			call	inc_count				; tixub 1 pulsi 
Din_2setlow:	bsf		Register1,.2
Din2_lstckl:	bcf		d2timeron
				goto	Din_3					; võta järgmine sisend
Din2high:bsf		lastinputs2,.2			; oli vist tõusev front, updateerime sisendite eelmist seisu
				btfss	dinpress2				; oli kõrge enne ?
				goto	Din_3					; ei, võta järgmine sisend
				btfsc	in_debounce				; kas debounce't teeb ?
				goto	Din_2stkh				; ei, arvestab kohe
				btfss	d2timeron				; lahti laskmise debounce käib?
				goto	Din2strt				; ei, paneme käima
				decfsz	din2tmr
				goto	Din_3					; võta järgmine sisend
				bcf		dinpress2				; loeme lahti lastuks
Din_2stkh:		bsf		dinstuck2				; blokeerime biti igal juhul
Din_2sethi:		bcf		Register1,.2
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
;			bcf		lastinputs2,.3			; oli vist langev front, updateerime sisendite eelmist seisu
				btfsc	in_debounce				; kas debounce't teeb ?
				goto	Din_3stk				; ei, arvestab kohe
				btfss	d3timeron				; jah, kas juba teame ?
				goto	Din3high1				; ei, käivita deb. timer kui vaja
				decfsz	din3tmr					; debouncetud?
				goto	Din_4					; ei veel, võta järgmine sisend
Din_3stk:		bsf		dinpress3				; jah, võtame arvesse
				bsf		dinstuck3				; blokeerime biti igal juhul
		bcf		lastinputs2,.3			; oli vist langev front, updateerime sisendite eelmist seisu
			btfss	muutus1,.3				; oli front või selline seis jätkub ?
			goto	Din_3setlow				; senine seis jätkub
			LFSR	.0,Loendi4+.3
			call	inc_count				; tixub 1 pulsi 
Din_3setlow:	bsf		Register1,.3
Din3_lstckl:	bcf		d3timeron
				goto	Din_4					; võta järgmine sisend
Din3high:bsf		lastinputs2,.3			; oli vist tõusev front, updateerime sisendite eelmist seisu
				btfss	dinpress3				; oli kõrge enne ?
				goto	Din_4					; ei, võta järgmine sisend
				btfsc	in_debounce				; kas debounce't teeb ?
				goto	Din_3stkh				; ei, arvestab kohe
				btfss	d3timeron				; lahti laskmise debounce käib?
				goto	Din3strt				; ei, paneme käima
				decfsz	din3tmr
				goto	Din_4					; võta järgmine sisend
				bcf		dinpress3				; loeme lahti lastuks
Din_3stkh:		bsf		dinstuck3				; blokeerime biti igal juhul
Din_3sethi:		bcf		Register1,.3
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
;			bcf		lastinputs2,.4			; oli vist langev front, updateerime sisendite eelmist seisu
				btfsc	in_debounce				; kas debounce't teeb ?
				goto	Din_4stk				; ei, arvestab kohe
				btfss	d4timeron				; jah, kas juba teame ?
				goto	Din4high1				; ei, käivita deb. timer kui vaja
				decfsz	din4tmr					; debouncetud?
				goto	Din_5					; ei veel, võta järgmine sisend
Din_4stk:		bsf		dinpress4				; jah, võtame arvesse
				bsf		dinstuck4				; blokeerime biti igal juhul
			bcf		lastinputs2,.4			; oli vist langev front, updateerime sisendite eelmist seisu
			btfss	muutus1,.4				; oli front või selline seis jätkub ?
			goto	Din_4setlow				; senine seis jätkub
			LFSR	.0,Loendi5+.3
			call	inc_count				; tixub 1 pulsi 
Din_4setlow:	bsf		Register1,.4
Din4_lstckl:	bcf		d4timeron
				goto	Din_5					; võta järgmine sisend
Din4high:bsf		lastinputs2,.4			; oli vist tõusev front, updateerime sisendite eelmist seisu
				btfss	dinpress4				; oli kõrge enne ?
				goto	Din_5					; ei, võta järgmine sisend
				btfsc	in_debounce				; kas debounce't teeb ?
				goto	Din_4stkh				; ei, arvestab kohe
				btfss	d4timeron				; lahti laskmise debounce käib?
				goto	Din4strt				; ei, paneme käima
				decfsz	din4tmr
				goto	Din_5					; võta järgmine sisend
				bcf		dinpress4				; loeme lahti lastuks
Din_4stkh:		bsf		dinstuck4				; blokeerime biti igal juhul
Din_4sethi:		bcf		Register1,.4
Din4_lstckh:	bcf		d4timeron
				goto	Din_5					; võta järgmine sisend
Din4high1:		btfsc	dinpress4
				goto	Din_5					; võta järgmine sisend
Din4strt:		movlw	debtime					; käivitame debounce taimeri
				movwf	din4tmr
				bsf		d4timeron
;**** sisendite debounce ***** - > sisend 5 (DIN plokist)
Din_5:			btfss	in_sticky
				goto	Din_5a
				btfsc	dinstuck5
				goto	Din_6
Din_5a:			btfsc	Din5					; sisend 5 madal ?
				goto	Din5high				; ei, kõrge, seedi seda
;				bcf		lastinputs2,.5			; oli vist langev front, updateerime sisendite eelmist seisu
				btfsc	in_debounce				; kas debounce't teeb ?
				goto	Din_5stk				; ei, arvestab kohe
				btfss	d5timeron				; jah, kas juba teame ?
				goto	Din5high1				; ei, käivita deb. timer kui vaja
				decfsz	din5tmr					; debouncetud?
				goto	Din_6					; ei veel, võta järgmine sisend
Din_5stk:		bsf		dinpress5				; jah, võtame arvesse
				bsf		dinstuck5				; blokeerime biti igal juhul
				bcf		lastinputs2,.5			; oli vist langev front, updateerime sisendite eelmist seisu
			btfss	muutus1,.5				; oli front või selline seis jätkub ?
			goto	Din_5setlow				; senine seis jätkub
			LFSR	.0,Loendi6+.3
			call	inc_count				; tixub 1 pulsi 
Din_5setlow:	bsf		Register1,.5
Din5_lstckl:	bcf		d5timeron
				goto	Din_6					; võta järgmine sisend
Din5high:		bsf		lastinputs2,.5			; oli vist tõusev front, updateerime sisendite eelmist seisu
				btfss	dinpress5				; oli kõrge enne ?
				goto	Din_6					; ei, võta järgmine sisend
				btfsc	in_debounce				; kas debounce't teeb ?
				goto	Din_5stkh				; ei, arvestab kohe
				btfss	d5timeron				; lahti laskmise debounce käib?
				goto	Din5strt				; ei, paneme käima
				decfsz	din5tmr
				goto	Din_6					; võta järgmine sisend
				bcf		dinpress5				; loeme lahti lastuks
Din_5stkh:		bsf		dinstuck5				; blokeerime biti igal juhul
Din_5sethi:		bcf		Register1,.5
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
;			bcf		lastinputs2,.6			; oli vist langev front, updateerime sisendite eelmist seisu
				btfsc	in_debounce				; kas debounce't teeb ?
				goto	Din_6stk				; ei, arvestab kohe
				btfss	d6timeron				; jah, kas juba teame ?
				goto	Din6high1				; ei, käivita deb. timer kui vaja
				decfsz	din6tmr					; debouncetud?
				goto	Din_7					; ei veel, võta järgmine sisend
Din_6stk:		bsf		dinpress6				; jah, võtame arvesse
				bsf		dinstuck6				; blokeerime biti igal juhul
			bcf		lastinputs2,.6			; oli vist langev front, updateerime sisendite eelmist seisu
			btfss	muutus1,.6				; oli front või selline seis jätkub ?
			goto	Din_6setlow				; senine seis jätkub
			LFSR	.0,Loendi7+.3
			call	inc_count				; tixub 1 pulsi 
Din_6setlow:	bsf		Register1,.6
Din6_lstckl:	bcf		d6timeron
				goto	Din_7					; võta järgmine sisend
Din6high:		bsf		lastinputs2,.6			; oli vist tõusev front, updateerime sisendite eelmist seisu
				btfss	dinpress6				; oli kõrge enne ?
				goto	Din_7					; ei, võta järgmine sisend
				btfsc	in_debounce				; kas debounce't teeb ?
				goto	Din_6stkh				; ei, arvestab kohe
				btfss	d6timeron				; lahti laskmise debounce käib?
				goto	Din6strt				; ei, paneme käima
				decfsz	din6tmr
				goto	Din_7					; võta järgmine sisend
				bcf		dinpress6				; loeme lahti lastuks
Din_6stkh:		bsf		dinstuck6				; blokeerime biti igal juhul
Din_6sethi:		bcf		Register1,.6
Din6_lstckh:	bcf		d6timeron
				goto	Din_7					; võta järgmine sisend
Din6high1:		btfsc	dinpress6
				goto	Din_7					; võta järgmine sisend
Din6strt:		movlw	debtime					; käivitame debounce taimeri
				movwf	din6tmr
				bsf		d6timeron
;**** sisendite debounce ***** - > sisend 7 (DIN plokist)
Din_7:			btfss	in_sticky
				goto	Din_7a
				btfsc	dinstuck7
				goto	Din_8
Din_7a:			btfsc	Din7					; sisend 7 madal ?
				goto	Din7high				; ei, kõrge, seedi seda
;			bcf		lastinputs2,.7			; oli vist langev front, updateerime sisendite eelmist seisu
				btfsc	in_debounce				; kas debounce't teeb ?
				goto	Din_7stk				; ei, arvestab kohe
				btfss	d7timeron				; jah, kas juba teame ?
				goto	Din7high1				; ei, käivita deb. timer kui vaja
				decfsz	din7tmr					; debouncetud?
				goto	Din_8					; ei veel, võta järgmine sisend
Din_7stk:		bsf		dinpress7				; jah, võtame arvesse
				bsf		dinstuck7				; blokeerime biti igal juhul
			bcf		lastinputs2,.7			; oli vist langev front, updateerime sisendite eelmist seisu
			btfss	muutus1,.7				; oli front või selline seis jätkub ?
			goto	Din_7setlow				; senine seis jätkub
			LFSR	.0,Loendi8+.3
			call	inc_count				; tixub 1 pulsi 
Din_7setlow:	bsf		Register1,.7
Din7_lstckl:	bcf		d7timeron
				goto	Din_8					; võta järgmine sisend
Din7high:		bsf		lastinputs2,.7			; oli vist tõusev front, updateerime sisendite eelmist seisu
				btfss	dinpress7				; oli kõrge enne ?
				goto	Din_8					; ei, võta järgmine sisend
				btfsc	in_debounce				; kas debounce't teeb ?
				goto	Din_7stkh				; ei, arvestab kohe
				btfss	d7timeron				; lahti laskmise debounce käib?
				goto	Din7strt				; ei, paneme käima
				decfsz	din7tmr
				goto	Din_8					; võta järgmine sisend
				bcf		dinpress7				; loeme lahti lastuks
Din_7stkh:		bsf		dinstuck7				; blokeerime biti igal juhul
Din_7sethi:		bcf		Register1,.7
Din7_lstckh:	bcf		d7timeron
				goto	Din_8					; võta järgmine sisend
Din7high1:		btfsc	dinpress7
				goto	Din_8					; võta järgmine sisend
Din7strt:		movlw	debtime					; käivitame debounce taimeri
				movwf	din7tmr
				bsf		d7timeron
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
				comf	Register271+.1,W		; lisa: XORime tulemuse sellega läbi
				xorwf	Registertemp5,F
;
				movf	Registertemp5,W
				xorwf	lastinputs1,W			; kas oli muutusi ?
				movwf	muutus1
;
				btfsc	Register275+.1,.0		; kas on sisend ?
				goto	Ain_0sethi				; eip, kirjutab nulli registrisse
				btfss	Register275+.0,.0		; kas on digisisend ?
				goto	Ain_0sethi				; eip, kirjutab nulli registrisse
				btfss	in_sticky
				goto	Ain_0a
				btfsc	ainstuck0
				goto	Ain_1
Ain_0a:			btfsc	Ana0					; sisend 0 madal ?
				goto	Ain0high				; ei, kõrge, seedi seda
;			bcf		lastinputs1,.0			; oli vist langev front, updateerime sisendite eelmist seisu
				btfsc	in_debounce				; kas debounce't teeb ?
				goto	Ain_0stk				; ei, arvestab kohe
				btfss	a0timeron				; jah, kas juba teame ?
				goto	Ain0high1				; ei, käivita deb. timer kui vaja
				decfsz	ain0tmr					; debouncetud?
				goto	Ain_1					; ei veel, võta järgmine sisend
Ain_0stk:		bsf		ainpress0				; jah, võtame arvesse
				bsf		ainstuck0				; blokeerime biti igal juhul
			bcf		lastinputs1,.0			; oli vist langev front, updateerime sisendite eelmist seisu
			btfss	muutus1,.0				; oli front või selline seis jätkub ?
			goto	Ain_0setlow				; senine seis jätkub
				LFSR	.0,Loendi9+.3
				call	inc_count				; tixub 1 pulsi 
;				btfss	sens9tmron				; tegevus loendina: JUBA DEBOUNCEME ?
;				bsf		sens9tmron 				; ei veel, siis taimer käima
Ain_0setlow:	bsf		Register1+.1,.0
Ain0_lstckl:	bcf		a0timeron
				goto	Ain_1					; võta järgmine sisend
Ain0high:	bsf		lastinputs1,.0			; oli vist tõusev front, updateerime sisendite eelmist seisu
				btfss	ainpress0				; oli kõrge enne ?
				goto	Ain_1					; ei, võta järgmine sisend
				btfsc	in_debounce				; kas debounce't teeb ?
				goto	Ain_0stkh				; ei, arvestab kohe
				btfss	a0timeron				; lahti laskmise debounce käib?
				goto	Ain0strt				; ei, paneme käima
				decfsz	ain0tmr
				goto	Ain_1					; võta järgmine sisend
				bcf		ainpress0				; loeme lahti lastuks
Ain_0stkh:		bsf		ainstuck0				; blokeerime biti igal juhul
Ain_0sethi:		bcf		Register1+.1,.0
Ain0_lstckh:	bcf		a0timeron
				goto	Ain_1					; võta järgmine sisend
Ain0high1:		btfsc	ainpress0
				goto	Ain_1					; võta järgmine sisend
Ain0strt:		movlw	debtime					; käivitame debounce taimeri
				movwf	ain0tmr
				bsf		a0timeron
;				goto	Ain_1					; võta järgmine sisend
;**** sisendite debounce ***** - > sisend 1 (AIN plokist)
Ain_1:			btfsc	Register275+.1,.1		; kas on sisend ?
				goto	Ain_1sethi				; eip, kirjutab nulli registrisse
				btfss	Register275+.0,.1		; kas on digisisend ?
				goto	Ain_1sethi				; eip, kirjutab nulli registrisse
				btfss	in_sticky
				goto	Ain_1a
				btfsc	ainstuck1
				goto	Ain_2
Ain_1a:			btfsc	Ana1					; sisend 1 madal ?
				goto	Ain1high				; ei, kõrge, seedi seda
;			bcf		lastinputs1,.1			; oli vist langev front, updateerime sisendite eelmist seisu
				btfsc	in_debounce				; kas debounce't teeb ?
				goto	Ain_1stk				; ei, arvestab kohe
				btfss	a1timeron				; jah, kas juba teame ?
				goto	Ain1high1				; ei, käivita deb. timer kui vaja
				decfsz	ain1tmr					; debouncetud?
				goto	Ain_2					; ei veel, võta järgmine sisend
Ain_1stk:		bsf		ainpress1				; jah, võtame arvesse
				bsf		ainstuck1				; blokeerime biti igal juhul
			bcf		lastinputs1,.1			; oli vist langev front, updateerime sisendite eelmist seisu
			btfss	muutus1,.1				; oli front või selline seis jätkub ?
			goto	Ain_1setlow				; senine seis jätkub
				LFSR	.0,Loendi10+.3
				call	inc_count				; tixub 1 pulsi 
;				btfss	sens10tmron				; tegevus loendina: JUBA DEBOUNCEME ?
;				bsf		sens10tmron				; ei veel, siis taimer käima
Ain_1setlow:	bsf		Register1+.1,.1
Ain1_lstckl:	bcf		a1timeron
				goto	Ain_2					; võta järgmine sisend
Ain1high:	bsf		lastinputs1,.1			; oli vist tõusev front, updateerime sisendite eelmist seisu
				btfss	ainpress1				; oli kõrge enne ?
				goto	Ain_2					; ei, võta järgmine sisend
				btfsc	in_debounce				; kas debounce't teeb ?
				goto	Ain_1stkh				; ei, arvestab kohe
				btfss	a1timeron				; lahti laskmise debounce käib?
				goto	Ain1strt				; ei, paneme käima
				decfsz	ain1tmr
				goto	Ain_2					; võta järgmine sisend
				bcf		ainpress1				; loeme lahti lastuks
Ain_1stkh:		bsf		ainstuck1				; blokeerime biti igal juhul
Ain_1sethi:		bcf		Register1+.1,.1
Ain1_lstckh:	bcf		a1timeron
				goto	Ain_2					; võta järgmine sisend
Ain1high1:		btfsc	ainpress1
				goto	Ain_2					; võta järgmine sisend
Ain1strt:		movlw	debtime					; käivitame debounce taimeri
				movwf	ain1tmr
				bsf		a1timeron
;				goto	Ain_2					; võta järgmine sisend
;**** sisendite debounce ***** - > sisend 2 (AIN plokist)
Ain_2:			btfsc	Register275+.1,.2		; kas on sisend ?
				goto	Ain_2sethi				; eip, kirjutab nulli registrisse
				btfss	Register275+.0,.2		; kas on digisisend ?
				goto	Ain_2sethi				; eip, kirjutab nulli registrisse
				btfss	in_sticky
				goto	Ain_2a
				btfsc	ainstuck2
				goto	Ain_3
Ain_2a:			btfsc	Ana2					; sisend 2 madal ?
				goto	Ain2high				; ei, kõrge, seedi seda
;			bcf		lastinputs1,.2			; oli vist langev front, updateerime sisendite eelmist seisu
				btfsc	in_debounce				; kas debounce't teeb ?
				goto	Ain_2stk				; ei, arvestab kohe
				btfss	a2timeron				; jah, kas juba teame ?
				goto	Ain2high1				; ei, käivita deb. timer kui vaja
				decfsz	ain2tmr					; debouncetud?
				goto	Ain_3					; ei veel, võta järgmine sisend
Ain_2stk:		bsf		ainpress2				; jah, võtame arvesse
				bsf		ainstuck2				; blokeerime biti igal juhul
			bcf		lastinputs1,.2			; oli vist langev front, updateerime sisendite eelmist seisu
			btfss	muutus1,.2				; oli front või selline seis jätkub ?
			goto	Ain_2setlow				; senine seis jätkub
				LFSR	.0,Loendi11+.3
				call	inc_count				; tixub 1 pulsi 
;				btfss	sens11tmron				; tegevus loendina: JUBA DEBOUNCEME ?
;				bsf		sens11tmron				; ei veel, siis taimer käima
Ain_2setlow:	bsf		Register1+.1,.2
Ain2_lstckl:	bcf		a2timeron
				goto	Ain_3					; võta järgmine sisend
Ain2high:	bsf		lastinputs1,.2			; oli vist tõusev front, updateerime sisendite eelmist seisu
				btfss	ainpress2				; oli kõrge enne ?
				goto	Ain_3					; ei, võta järgmine sisend
				btfsc	in_debounce				; kas debounce't teeb ?
				goto	Ain_2stkh				; ei, arvestab kohe
				btfss	a2timeron				; lahti laskmise debounce käib?
				goto	Ain2strt				; ei, paneme käima
				decfsz	ain2tmr
				goto	Ain_3					; võta järgmine sisend
				bcf		ainpress2				; loeme lahti lastuks
Ain_2stkh:		bsf		ainstuck2				; blokeerime biti igal juhul
Ain_2sethi:		bcf		Register1+.1,.2
Ain2_lstckh:	bcf		a2timeron
				goto	Ain_3					; võta järgmine sisend
Ain2high1:		btfsc	ainpress2
				goto	Ain_3					; võta järgmine sisend
Ain2strt:		movlw	debtime					; käivitame debounce taimeri
				movwf	ain2tmr
				bsf		a2timeron
;				goto	Ain_3					; võta järgmine sisend
;**** sisendite debounce ***** - > sisend 3 (AIN plokist)
Ain_3:			btfsc	Register275+.1,.3		; kas on sisend ?
				goto	Ain_3sethi				; eip, kirjutab nulli registrisse
				btfss	Register275+.0,.3		; kas on digisisend ?
				goto	Ain_3sethi				; eip, kirjutab nulli registrisse
				btfss	in_sticky
				goto	Ain_3a
				btfsc	ainstuck3
				goto	Ain_4
Ain_3a:			btfsc	Ana3					; sisend 3 madal ?
				goto	Ain3high				; ei, kõrge, seedi seda
;			bcf		lastinputs1,.3			; oli vist langev front, updateerime sisendite eelmist seisu
				btfsc	in_debounce				; kas debounce't teeb ?
				goto	Ain_3stk				; ei, arvestab kohe
				btfss	a3timeron				; jah, kas juba teame ?
				goto	Ain3high1				; ei, käivita deb. timer kui vaja
				decfsz	ain3tmr					; debouncetud?
				goto	Ain_4					; ei veel, võta järgmine sisend
Ain_3stk:		bsf		ainpress3				; jah, võtame arvesse
				bsf		ainstuck3				; blokeerime biti igal juhul
			bcf		lastinputs1,.3			; oli vist langev front, updateerime sisendite eelmist seisu
			btfss	muutus1,.3				; oli front või selline seis jätkub ?
			goto	Ain_3setlow				; senine seis jätkub
				LFSR	.0,Loendi12+.3
				call	inc_count				; tixub 1 pulsi 
;				btfss	sens12tmron				; tegevus loendina: JUBA DEBOUNCEME ?
;				bsf		sens12tmron				; ei veel, siis taimer käima
Ain_3setlow:	bsf		Register1+.1,.3
Ain3_lstckl:	bcf		a3timeron
				goto	Ain_4					; võta järgmine sisend
Ain3high:	bsf		lastinputs1,.3			; oli vist tõusev front, updateerime sisendite eelmist seisu
				btfss	ainpress3				; oli kõrge enne ?
				goto	Ain_4					; ei, võta järgmine sisend
				btfsc	in_debounce				; kas debounce't teeb ?
				goto	Ain_3stkh				; ei, arvestab kohe
				btfss	a3timeron				; lahti laskmise debounce käib?
				goto	Ain3strt				; ei, paneme käima
				decfsz	ain3tmr
				goto	Ain_4					; võta järgmine sisend
				bcf		ainpress3				; loeme lahti lastuks
Ain_3stkh:		bsf		ainstuck3				; blokeerime biti igal juhul
Ain_3sethi:		bcf		Register1+.1,.3
Ain3_lstckh:	bcf		a3timeron
				goto	Ain_4					; võta järgmine sisend
Ain3high1:		btfsc	ainpress3
				goto	Ain_4					; võta järgmine sisend
Ain3strt:		movlw	debtime					; käivitame debounce taimeri
				movwf	ain3tmr
				bsf		a3timeron
;				goto	Ain_4					; võta järgmine sisend
;**** sisendite debounce ***** - > sisend 4 (AIN plokist)
Ain_4:			btfsc	Register275+.1,.4		; kas on sisend ?
				goto	Ain_4sethi				; eip, kirjutab nulli registrisse
				btfss	Register275+.0,.4		; kas on digisisend ?
				goto	Ain_4sethi				; eip, kirjutab nulli registrisse
				btfss	in_sticky
				goto	Ain_4a
				btfsc	ainstuck4
				goto	Ain_5
Ain_4a:			btfsc	Ana4					; sisend 4 madal ?
				goto	Ain4high				; ei, kõrge, seedi seda
;			bcf		lastinputs1,.4			; oli vist langev front, updateerime sisendite eelmist seisu
				btfsc	in_debounce				; kas debounce't teeb ?
				goto	Ain_4stk				; ei, arvestab kohe
				btfss	a4timeron				; jah, kas juba teame ?
				goto	Ain4high1				; ei, käivita deb. timer kui vaja
				decfsz	ain4tmr					; debouncetud?
				goto	Ain_5					; ei veel, võta järgmine sisend
Ain_4stk:		bsf		ainpress4				; jah, võtame arvesse
				bsf		ainstuck4				; blokeerime biti igal juhul
			bcf		lastinputs1,.4			; oli vist langev front, updateerime sisendite eelmist seisu
			btfss	muutus1,.4				; oli front või selline seis jätkub ?
			goto	Ain_4setlow				; senine seis jätkub
				LFSR	.0,Loendi13+.3
				call	inc_count				; tixub 1 pulsi 
;				btfss	sens13tmron				; tegevus loendina: JUBA DEBOUNCEME ?
;				bsf		sens13tmron				; ei veel, siis taimer käima
Ain_4setlow:	bsf		Register1+.1,.4
Ain4_lstckl:	bcf		a4timeron
				goto	Ain_5					; võta järgmine sisend
Ain4high:	bsf		lastinputs1,.4			; oli vist tõusev front, updateerime sisendite eelmist seisu
				btfss	ainpress4				; oli kõrge enne ?
				goto	Ain_5					; ei, võta järgmine sisend
				btfsc	in_debounce				; kas debounce't teeb ?
				goto	Ain_4stkh				; ei, arvestab kohe
				btfss	a4timeron				; lahti laskmise debounce käib?
				goto	Ain4strt				; ei, paneme käima
				decfsz	ain4tmr
				goto	Ain_5					; võta järgmine sisend
				bcf		ainpress4				; loeme lahti lastuks
Ain_4stkh:		bsf		ainstuck4				; blokeerime biti igal juhul
Ain_4sethi:		bcf		Register1+.1,.4
Ain4_lstckh:	bcf		a4timeron
				goto	Ain_5					; võta järgmine sisend
Ain4high1:		btfsc	ainpress4
				goto	Ain_5					; võta järgmine sisend
Ain4strt:		movlw	debtime					; käivitame debounce taimeri
				movwf	ain4tmr
				bsf		a4timeron
;				goto	Ain_5					; võta järgmine sisend
;**** sisendite debounce ***** - > sisend 5 (AIN plokist)
Ain_5:			btfsc	Register275+.1,.5		; kas on sisend ?
				goto	Ain_5sethi				; eip, kirjutab nulli registrisse
				btfss	Register275+.0,.5		; kas on digisisend ?
				goto	Ain_5sethi				; eip, kirjutab nulli registrisse
				btfss	in_sticky
				goto	Ain_5a
				btfsc	ainstuck5
				goto	Ain_6
Ain_5a:			btfsc	Ana5					; sisend 5 madal ?
				goto	Ain5high				; ei, kõrge, seedi seda
;			bcf		lastinputs1,.5			; oli vist langev front, updateerime sisendite eelmist seisu
				btfsc	in_debounce				; kas debounce't teeb ?
				goto	Ain_5stk				; ei, arvestab kohe
				btfss	a5timeron				; jah, kas juba teame ?
				goto	Ain5high1				; ei, käivita deb. timer kui vaja
				decfsz	ain5tmr					; debouncetud?
				goto	Ain_6					; ei veel, võta järgmine sisend
Ain_5stk:		bsf		ainpress5				; jah, võtame arvesse
				bsf		ainstuck5				; blokeerime biti igal juhul
			bcf		lastinputs1,.5			; oli vist langev front, updateerime sisendite eelmist seisu
			btfss	muutus1,.5				; oli front või selline seis jätkub ?
			goto	Ain_5setlow				; senine seis jätkub
				LFSR	.0,Loendi14+.3
				call	inc_count				; tixub 1 pulsi 
;				btfss	sens14tmron				; tegevus loendina: JUBA DEBOUNCEME ?
;				bsf		sens14tmron				; ei veel, siis taimer käima
Ain_5setlow:	bsf		Register1+.1,.5
Ain5_lstckl:	bcf		a5timeron
				goto	Ain_6					; võta järgmine sisend
Ain5high:	bsf		lastinputs1,.5			; oli vist tõusev front, updateerime sisendite eelmist seisu
				btfss	ainpress5				; oli kõrge enne ?
				goto	Ain_6					; ei, võta järgmine sisend
				btfsc	in_debounce				; kas debounce't teeb ?
				goto	Ain_5stkh				; ei, arvestab kohe
				btfss	a5timeron				; lahti laskmise debounce käib?
				goto	Ain5strt				; ei, paneme käima
				decfsz	ain5tmr
				goto	Ain_6					; võta järgmine sisend
				bcf		ainpress5				; loeme lahti lastuks
Ain_5stkh:		bsf		ainstuck5				; blokeerime biti igal juhul
Ain_5sethi:		bcf		Register1+.1,.5
Ain5_lstckh:	bcf		a5timeron
				goto	Ain_6					; võta järgmine sisend
Ain5high1:		btfsc	ainpress5
				goto	Ain_6					; võta järgmine sisend
Ain5strt:		movlw	debtime					; käivitame debounce taimeri
				movwf	ain5tmr
				bsf		a5timeron
;				goto	Ain_6					; võta järgmine sisend
;**** sisendite debounce ***** - > sisend 6 (AIN plokist)
Ain_6:			btfsc	Register275+.1,.6		; kas on sisend ?
				goto	Ain_6sethi				; eip, kirjutab nulli registrisse
				btfss	Register275+.0,.6		; kas on digisisend ?
				goto	Ain_6sethi				; eip, kirjutab nulli registrisse
				btfss	in_sticky
				goto	Ain_6a
				btfsc	ainstuck6
				goto	Ain_7
Ain_6a:			btfsc	Ana6					; sisend 6 madal ?
				goto	Ain6high				; ei, kõrge, seedi seda
;			bcf		lastinputs1,.6			; oli vist langev front, updateerime sisendite eelmist seisu
				btfsc	in_debounce				; kas debounce't teeb ?
				goto	Ain_6stk				; ei, arvestab kohe
				btfss	a6timeron				; jah, kas juba teame ?
				goto	Ain6high1				; ei, käivita deb. timer kui vaja
				decfsz	ain6tmr					; debouncetud?
				goto	Ain_7					; ei veel, võta järgmine sisend
Ain_6stk:		bsf		ainpress6				; jah, võtame arvesse
				bsf		ainstuck6				; blokeerime biti igal juhul
			bcf		lastinputs1,.6			; oli vist langev front, updateerime sisendite eelmist seisu
			btfss	muutus1,.6				; oli front või selline seis jätkub ?
			goto	Ain_6setlow				; senine seis jätkub
				LFSR	.0,Loendi15+.3
				call	inc_count				; tixub 1 pulsi 
;				btfss	sens15tmron				; tegevus loendina: JUBA DEBOUNCEME ?
;				bsf		sens15tmron				; ei veel, siis taimer käima
Ain_6setlow:	bsf		Register1+.1,.6
Ain6_lstckl:	bcf		a6timeron
				goto	Ain_7					; võta järgmine sisend
Ain6high:	bsf		lastinputs1,.6			; oli vist tõusev front, updateerime sisendite eelmist seisu
				btfss	ainpress6				; oli kõrge enne ?
				goto	Ain_7					; ei, võta järgmine sisend
				btfsc	in_debounce				; kas debounce't teeb ?
				goto	Ain_6stkh				; ei, arvestab kohe
				btfss	a6timeron				; lahti laskmise debounce käib?
				goto	Ain6strt				; ei, paneme käima
				decfsz	ain6tmr
				goto	Ain_7					; võta järgmine sisend
				bcf		ainpress6				; loeme lahti lastuks
Ain_6stkh:		bsf		ainstuck6				; blokeerime biti igal juhul
Ain_6sethi:		bcf		Register1+.1,.6
Ain6_lstckh:	bcf		a6timeron
				goto	Ain_7					; võta järgmine sisend
Ain6high1:		btfsc	ainpress6
				goto	Ain_7					; võta järgmine sisend
Ain6strt:		movlw	debtime					; käivitame debounce taimeri
				movwf	ain6tmr
				bsf		a6timeron
;				goto	Ain_7					; võta järgmine sisend
;**** sisendite debounce ***** - > sisend 7 (AIN plokist)
Ain_7:			btfsc	Register275+.1,.7		; kas on sisend ?
				goto	Ain_7sethi				; eip, kirjutab nulli registrisse
				btfss	Register275+.0,.7		; kas on digisisend ?
				goto	Ain_7sethi				; eip, kirjutab nulli registrisse
				btfss	in_sticky
				goto	Ain_7a
				btfsc	ainstuck7
				goto	Ain_8
Ain_7a:			btfsc	Ana7					; sisend 7 madal ?
				goto	Ain7high				; ei, kõrge, seedi seda
;			bcf		lastinputs1,.7			; oli vist langev front, updateerime sisendite eelmist seisu
				btfsc	in_debounce				; kas debounce't teeb ?
				goto	Ain_7stk				; ei, arvestab kohe
				btfss	a7timeron				; jah, kas juba teame ?
				goto	Ain7high1				; ei, käivita deb. timer kui vaja
				decfsz	ain7tmr					; debouncetud?
				goto	Ain_8					; ei veel, võta järgmine sisend
Ain_7stk:		bsf		ainpress7				; jah, võtame arvesse
				bsf		ainstuck7				; blokeerime biti igal juhul
			bcf		lastinputs1,.7			; oli vist langev front, updateerime sisendite eelmist seisu
			btfss	muutus1,.7				; oli front või selline seis jätkub ?
			goto	Ain_7setlow				; senine seis jätkub
				LFSR	.0,Loendi16+.3
				call	inc_count				; tixub 1 pulsi 
;				btfss	sens16tmron				; tegevus loendina: JUBA DEBOUNCEME ?
;				bsf		sens16tmron				; ei veel, siis taimer käima
Ain_7setlow:	bsf		Register1+.1,.7
Ain7_lstckl:	bcf		a7timeron
				goto	Ain_8					; võta järgmine sisend
Ain7high:	bsf		lastinputs1,.7			; oli vist tõusev front, updateerime sisendite eelmist seisu
				btfss	ainpress7				; oli kõrge enne ?
				goto	Ain_8					; ei, võta järgmine sisend
				btfsc	in_debounce				; kas debounce't teeb ?
				goto	Ain_7stkh				; ei, arvestab kohe
				btfss	a7timeron				; lahti laskmise debounce käib?
				goto	Ain7strt				; ei, paneme käima
				decfsz	ain7tmr
				goto	Ain_8					; võta järgmine sisend
				bcf		ainpress7				; loeme lahti lastuks
Ain_7stkh:		bsf		ainstuck7				; blokeerime biti igal juhul
Ain_7sethi:		bcf		Register1+.1,.7
Ain7_lstckh:	bcf		a7timeron
				goto	Ain_8					; võta järgmine sisend
Ain7high1:		btfsc	ainpress7
				goto	Ain_8					; võta järgmine sisend
Ain7strt:		movlw	debtime					; käivitame debounce taimeri
				movwf	ain7tmr
				bsf		a7timeron
Ain_8:
T1int_end:
				goto	Int_2
;===============================================================================
; ********************** funktsioonid ******************************************
;===============================================================================
;===============================================================================
; ********************** põhiluup **********************************************
;===============================================================================
main:			call	init					; prose setup
				call	init_analoog			; analoogpingete mõõtmise init
;--- lubame intsid -------------
				banksel	PIE1
				movlw	B'00100001'		 		; luba vaid ser. receive ja T1 intsid
				movwf	PIE1
				banksel	.0
				movf	PORTB,W				
				bsf	INTCON,RBIE
;	bsf	INTCON,T0IE
				movlw	B'00010000'				; katskestusi ei ole esinenud, RS232 porti ei puudu: read-only
				movwf	PIR1	
				banksel	IOCB
				movlw	0xFF
				movwf	IOCB					; lubame kõik pordi B muutuse katckestused
				banksel	.0
				clrwdt

				movlw	0x04					; Capture igal langeval frondil
				banksel	CCP2CON
				movwf	CCP2CON
				banksel	PIE3
				bsf		PIE3,CCP2IE
				banksel	.0
				movlw	0x04
				movwf	IPR3					; CCP2 kõrge prioriteediga
				clrf	IPR1					; kõik muud intsid on madala prioriteediga
				clrf	IPR2
				clrf	IPR4
				clrf	IPR5

				clrf	INTCON2
				bsf		INTCON2,.7				; B-pullupid maha

				bsf		RCON,IPEN				; lubame katkestuste prioriteedid

				bcf		INTCON,TMR0IF			; debuggeri jaox ainult !
				bsf		INTCON,PEIE			
				bsf		INTCON,GIE				; lubame kindral-katckestused
; jupitamise debug
;				bcf		INTCON,TMR0IE			; see rsk solgibki saate ära (viide iga baidi järel)
;
tsykkel:		bcf		PIE1,ADIE
				bcf		PIR1,TXIF				; katkestuse nõue maha
				btfss	cmd_ok					; oli valiidne käsk ?
				goto	adc000					; eip, lase edasi
;----------
				btfss	temper					; aeg temperatuuri mõõta ?
				goto	aaa
				call	GetT					; jepp
				bcf		temper					; mõõtmine tehtud
				movlw	_10sekundiaeg
				movwf	_10sekunditmr
aaa:			call	command					; jah - töötle ja täida 
				call	reset_ser				; tehtud !
				bcf		cmd_ok					; tehtud !

				btfsc	special0				; olid erikäsud ?
				goto	special_cmd0			; 
				btfsc	special1
				goto	special_cmd1	
				goto	aab
special_cmd0:	bcf		INTCON,GIE
				bcf		INTCON,PEIE			
				bcf		RCON,IPEN
		bcf		special0			
				LFSR	.1,Dallas1				; hakkame andureid detectima: DS1820 kontrollbaidid näidaku et andurid puuduvad (0x1000)
				movlw	.9
				movwf	DS1820found
ds1820luup:		movlw	0x10
				movwf	POSTINC1
				movlw	0x00
				movwf	POSTINC1
				decfsz	DS1820found
				goto	ds1820luup
				LFSR	.1,Dallas1wa
				movlw	.72
				movwf	DS1820found
ds1820luup1:	movlw	0x00
				movwf	POSTINC1
				decfsz	DS1820found
				goto	ds1820luup1

				LFSR	.1,DS2438_1				; DS2438 kontrollbaidid näidaku samuti et andurid puuduvad (0x1000)
				movlw	.9*.4
				movwf	DS1820found
ds2438luup:		movlw	0x10
				movwf	POSTINC1
				movlw	0x00
				movwf	POSTINC1
				decfsz	DS1820found
				goto	ds2438luup
				LFSR	.1,DS24381wa
				movlw	.72
				movwf	DS1820found
ds1820luup2:	movlw	0x00
				movwf	POSTINC1
				decfsz	DS1820found
				goto	ds1820luup2

				call	discovery
				clrwdt                    		; WDT nullida 
				bsf		RCON,IPEN				; lubame katkestuste prioriteedid
				bsf		INTCON,PEIE			
				bsf		INTCON,GIE
				goto	aab
special_cmd1:	bsf		reset1pulseon			; märgime ära
				bcf		n_reset1				; ja nüüd annab saapaga ... ! -> inversioonis
				movff	Register276,WREG		; taasta side kadumise viiteaeg
				movwf	reset1dlytmrH
				movff	Register276+.1,WREG
				movwf	reset1dlytmrL
	
aab:			bcf		PIR1,RC1IF
				banksel	PIE1
				bsf		PIE1,RC1IE				; lubame uue käsu vastuvõttu
				banksel	.0
adc000:	
				banksel	PIE1
				bsf		PIE1,RC1IE				; lubame uue käsu vastuvõttu
				banksel	.0

				LFSR	.0,Register2			; ADC muundamine - käib kõik kanalid läbi
				clrf	meascount
adcchan0:		btfsc	Register275+.1,.0		; kui see bitt on väljund, kirjutame tulemuseks 0
				goto	adc0					; ongi
				btfss	Register275+.0,.0		; kui see bitt on digisisend, kirjutame tulemuseks 0
				goto	adc00					; sobib, muundame
adc0:			movlw	0x00
				movff	WREG,Register2
				movff	WREG,Register2+.1
				goto	adcchan1				; võta järgmine kanal (sisend)
adc00:			clrwdt
				banksel	ADCON0
				movlw	chan1					; on anasisend, muundame
				movwf	ADCON0
				call	measure

adcchan1:		incf	meascount,F				; register 3
				LFSR	.0,Register3
				btfsc	Register275+.1,.1		
				goto	adc1					
				btfss	Register275+.0,.1		
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
				btfss	Register275+.0,.2		
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
				btfss	Register275+.0,.3		
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
				btfss	Register275+.0,.4		
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
				btfss	Register275+.0,.5		
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
				btfss	Register275+.0,.6		
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
				btfss	Register275+.0,.7		
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
incrpointer:	movlw	.7						; viitab järgmise kanali tulemustele
				addwf	FSR1L,F
				btfsc	CARRY
				incf	FSR1H,F
				return
;===============================================================================
; ************** analoogpingete mõõtmine ***************************************
;===============================================================================
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
				movf	INDF1,W					; 10 korda mõõdetud ?
				addlw	.0
				btfsc	ZERO
				goto	keskmista				; jah, arvuta tulemus
				movlw	sampletime				; 4* 69,625us=278,5us
				movwf	sampledly
sdly:			decfsz	sampledly
				goto	sdly
				movlw	sampletime				
				movwf	sampledly
sdly1:			decfsz	sampledly
				goto	sdly
				movlw	sampletime				
				movwf	sampledly
sdly2:			decfsz	sampledly
				goto	sdly2
				movlw	sampletime				
				movwf	sampledly
sdly3:			decfsz	sampledly
				goto	sdly3
				banksel	ADCON0
				bsf		ADCON0,GO
meas1:			BTFSC 	ADCON0,GO 
				goto	meas1
				BANKSEL ADRESH 
				movlw	0x0F					; kui >4095 siis näitu ei salvesta ( on jama !)
				cpfsgt	ADRESH
				goto	meas2					; on OK	
				banksel .0
				movf	POSTINC1,W				; on liiga suur number, suurendame vaid tulemuse mäluviita
				movf	POSTINC1,W
				return
meas2:			decf	POSTINC1,F				; tulemus OK, vähenda keskmistamise kordade loendit
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
				call	comparemin				; võrdleme min. väärtusega: kui A < Min,  vaiksem = 1
				btfss	vaiksem
				goto	meas3					; ei ole <, senine min jääb kehtima
				movf	POSTDEC1,W				; tulemus < Min, seivime ta uueks min'iks	
				movf	POSTDEC1,W				; selleks viita tagasi min väärtusele
				banksel	ADRESH
				movf	ADRESH,W
				banksel	.0
				movwf	POSTINC1
				banksel	ADRESL
				movf	ADRESL,W
				banksel	.0
				movwf	POSTINC1
meas3:			call	comparemax				; võrdleme max. väärtusega
				btfss	suurem					; kui A < Max,  Cy = 1
				goto	meas4					; on <, senine max jääb kehtima
				movf	POSTDEC1,W				; tulemus > Max, seivime ta uueks max'iks	
				movf	POSTDEC1,W				; selleks viita tagasi max väärtusele
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
				call	sub1616max				; lahuta ka maksimumväärtus
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
				movff	POSTINC1,POSTINC0		; tõsta valmis tulemus oma registrisse
				movff	POSTINC1,POSTINC0		
				movff	FSR2L,FSR1L				; nullime mõõtmistsükli väärtused
				movff	FSR2H,FSR1H
				movf	POSTDEC1,W
				movlw	mõõtmisi
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
;===============================================================================
comparemax:		nop
				banksel	ADRESH
				movf   	ADRESH,W		        ; pinge võrdlemine Max-ga
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
				movf   	ADRESH,W		        ; pinge võrdlemine Min-ga
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
   				goto  	compare_eq     		   	; võrdsed
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
;===============================================================================
;       SourceH:SourceL - Number to be subtracted
;       Carry - NOT( Borrow to be subtracted )
;       DestH:DestL - Number to be subtracted FROM
;Out    DestH:DestL - Result
;       Carry - NOT( Borrow result)
; Dest=Dest-Source
;===============================================================================
sub1616min:		movff	POSTINC1,DestL			; summa registri sisust lahutame
				movff	POSTINC1,DestH			; NB! Summa on LSB, MSB järjekorras
				movff	POSTINC1,SourceH		; min registri sisu
				movff	POSTINC1,SourceL
				goto	sub1616
sub1616max:		movff	POSTINC1,DestL			; summa registri sisust lahutame
				movff	POSTINC1,DestH			; NB! Summa on LSB, MSB järjekorras
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
;===============================================================================
; MB_CRC16 -	Will calculate 16bit CRC for MODBUS RTU Packets
;
;		Input: 	Byte for CRC in W
;		Output:	Original byte in W, 
;			CRC16_HI and CRC16_LO new value of CRC16
;===============================================================================
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
				btfsc	Register273+.1,.7		; bit7 paarsus. 0=EVEN,1=NO PARITY
				goto	snd_exit
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
				banksel	TXSTA1
				bcf		TXSTA1,TX9D
				btfsc	CARRY
				bsf		TXSTA1,TX9D
				banksel	.0
snd_exit:
snd_exita:		btfss   PIR1,TXIF     			; saatja valmis ?   
			    goto    snd_exita
				movf	Char,W					; saadetav bait meelde tuletada
				banksel	TXREG1
				movwf   TXREG1    				; saada!
				banksel	TXSTA1
snd_exit1:		btfss	TXSTA1,TRMT				; kas saatja nihkeregister tühi (bait prosest väljas)?
				goto	snd_exit1
				banksel	.0
				movf	Char,W					; taasta saadetav
			    return
;===============================================================================
setup_serial:	movff	Register273+.1,WREG	
				andlw	0x07
				movff	WREG,Register273
				movlw	.71						; baudrate = (9600 @ 11.0592 MHz)
				banksel	SPBRG1	
				movwf	SPBRG1		
				banksel	.0
				movff	Register273,WREG
				sublw	.1						; kas on 9600 ?
				btfsc	ZERO
				goto	initser_par				; jah
				movlw	.35						; baudrate = (19200 @ 11.0592 MHz)
				banksel	SPBRG1	
				movwf	SPBRG1		
				banksel	.0
				movff	Register273,WREG
				sublw	.2						; kas on 19200 ?
				btfsc	ZERO
				goto	initser_par				; jah
				movlw	.17						; baudrate = (38400 @ 11.0592 MHz)
				banksel	SPBRG1	
				movwf	SPBRG1		
				banksel	.0
				movff	Register273,WREG
				sublw	.3						; kas on 38400 ?
				btfsc	ZERO
				goto	initser_par				; jah
				movlw	.11						; baudrate = (57600 @ 11.0592 MHz)
				banksel	SPBRG1	
				movwf	SPBRG1		
				banksel	.0
				movff	Register273,WREG
				sublw	.4						; kas on 57600 ?
				btfsc	ZERO
				goto	initser_par				; jah
				movlw	.5						; baudrate = (115200 @ 11.0592 MHz)
				banksel	SPBRG1	
				movwf	SPBRG1		
				banksel	.0
				movff	Register273,WREG
				sublw	.5						; kas on 115200 ?
				btfsc	ZERO
				goto	initser_par				; jah
setup_serial0:
				movlw	.35						; mingi kamm, võtab baudrate = (19200 @ 11.0592 MHz)
				banksel	SPBRG1	
				movwf	SPBRG1		
				banksel	.0
				movff	Register273+.1,WREG		; parandame vea: võtab vaikimisi seriali seaded aga ei muuda debounce ja sticky bitte ! Samuti ei näpi Wiegandi bitte !!!
				andlw	0x78
				movff	WREG,Register273
				movlw	defaultserial
				andlw	0x07
				addwf	Register273,W
				movff	WREG,Register273+.1			
initser_par:	movlw	0x00
				movff	WREG,Register273		; selle solkisime ää, nüüd nulli
;				movlw	B'01100111'				; paarsuse kalkuleerimine - eeldame: 9 bitine saade
				movlw	B'01100011'				; paarsuse kalkuleerimine - eeldame: 9 bitine saade, SYNC=BRGH=BRG16=0
				btfsc	Register273+.1,.7		; paarsus even (0) või puudub (1) ?
;				movlw	B'00100110'				; paarsus puudub: 8 bitine saade
				movlw	B'00100010'				; paarsus puudub: 8 bitine saade, SYNC=BRGH=BRG16=0
				banksel	TXSTA1
				movwf	TXSTA1
				banksel	.0
				movlw	B'11010000'				; sama RCSTA jaoks: eeldame paarsust st. 9-bitist saadet
				btfsc	Register273+.1,.7		
				movlw	B'10010000'			
				banksel	RCSTA1
				movwf	RCSTA1
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
				call	Read_EEPROM				; Analoog-Pull-up'ide seis ehk anaväljundite seis
				movff	WREG,Register272+.1		
				movff	WREG,Register0+.1		; ja väljundite seisu näitavasse registrisse (NB! ainult stardil)
				call	Read_EEPROM					
				movff	WREG,Register272+.0		; Digital-Pull-up'ide seis
				movff	WREG,Register0			; ja väljundite seisu näitavasse registrisse
				movwf	DoutPort				; ja kohe väljunditesse kah (NB! ainult stardil)
				call	Read_EEPROM				; seriali parameetrid	
				movff	WREG,Register273+.1
				clrf	Register273	
				call	Read_EEPROM				; ANA-pordi (UIO) suund, 1= väljund, bitthaaval)	
				movff	WREG,Register275+.1		
				call	Read_EEPROM				; analoogpordi seisund stardil - analoog või digi. 1= digi
				movff	WREG,Register275+.0
				call	Read_EEPROM				; seadme tüüp	
				movff	WREG,Register256+.1
				movlw	0x00
				movff	WREG,Register256
				call	Read_EEPROM				; firmware nr HIGH
				movff	WREG,Register257+.0
				call	Read_EEPROM				; firmware nr LOW
				movff	WREG,Register257+.1

; *** resetid ****

; Muudatus:
; 276 RW reset1 (USB), rakendumise aeg 16bit, sekundites
; 277 RW reset1 (USB), pulsi kestus 16 bit, sekundites
; 278 RW reset2 (Phone Power Button), rakendumise aeg 16bit, sekundites
; 279 RW reset2 (Phone Power Button), pulsi kestus 16 bit, sekundites
				call	Read_EEPROM				; reset 1 ajad: side kadumise viiteaeg
				movff	WREG,Register276
				movwf	reset1dlytmrH
				call	Read_EEPROM					
				movff	WREG,Register276+.1
				movwf	reset1dlytmrL
				call	Read_EEPROM				; reseti pulsi kestus							
				movff	WREG,Register277
				movwf	reset1pulsetmrH
				call	Read_EEPROM					
				movff	WREG,Register277+.1	
				movwf	reset1pulsetmrL
				call	Read_EEPROM				; reset 2 ajad: side kadumise viiteaeg
				movff	WREG,Register278
				movwf	reset2dlytmrH
				call	Read_EEPROM					
				movff	WREG,Register278+.1
				movwf	reset2dlytmrL
				call	Read_EEPROM				; reseti pulsi kestus							
				movff	WREG,Register279
				movwf	reset2pulsetmrH
				call	Read_EEPROM					
				movff	WREG,Register279+.1	
				movwf	reset2pulsetmrL
; *** XORimine ****
				call	Read_EEPROM				; juhitav aktiivne nivoo DIGI-pordi sisendis
				movff	WREG,Register271+.0
				call	Read_EEPROM				; juhitav aktiivne nivoo ANA-pordi sisendis
				movff	WREG,Register271+.1
; *** ADC ****
				call	Read_EEPROM				; ADC registri ADCON1 bitid
				andlw	0x30					; lubame vaid Vref-i puudutavaid bitte
				movff	WREG,Register270+.1
				BANKSEL ADCON1 
				MOVWF 	ADCON1 					; konfime kohe ADC ka ära
				banksel	.0
				call	Read_EEPROM				; PWMi lubatuse bitid
				movff	WREG,Register270
;				bcf		pwmenabled
;				btfsc	Register270,.0
;				bsf		pwmenabled
;				bcf		INTCON,TMR0IE
;				btfsc	pwmenabled
				bsf		INTCON,TMR0IE
; *** PWM ****
				call	Read_EEPROM				; PWMi perioodi register
				movff	WREG,Register150+.0
				call	Read_EEPROM			
				movff	WREG,Register150+.1
				call	Read_EEPROM				; PWMi konfi register
				movff	WREG,Register151+.0
				call	Read_EEPROM			
				movff	WREG,Register151+.1
; *** Dallase adr. lukustamine ***
				movlw	LOW(e_reg699)			; loe reg. 699-st lukustamise luba või keeld
				banksel	EEADR
				movwf	EEADR
				clrf	EEADRH
				call	Read_EEPROM					
				movff	WREG,Register699+.0
				call	Read_EEPROM					
				movff	WREG,Register699+.1
; *** lõpetame .... 
				call	reset_pwm				; initsialiseerime PWMi
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
		bsf		T0CON,TMR0ON
				goto	rp1
reset_pwm1:		bsf		T0CON,TMR0ON			; PWMi taimer käima
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
reset_chanls1:	btfsc	INDF0,.7				; ühekordse pulsi kestust ei taasta, olgu 0
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
				return
reset_per:		movlw	0x00
				movff	WREG,periodtick+.0		; nulli perioodi loendi et periood saax kohe alata
				movff	WREG,periodtick+.1
				movff	WREG,periodtick+.2
				movlw	T0reso;.0				
				movwf	TMR0L
				LFSR	.0,Register151+.1
				movlw	0x00
				movwf	INDF0
				movf	POSTDEC0,W				; järgmised 8 kanalit
				movlw	0x00
				movwf	INDF0
				return
;===============================================================================
; **** Salvestab parameetrid EEPROMisse ja kirjutab süsteemi *******************
;===============================================================================
Save_Setup:		movlw	LOW(e_ADR)				; loe oma aadress
				banksel	EEADR
				movwf	EEADR
				clrf	EEADRH
				movff	Register274+.1,WREG		; aadress
				call	Wr_EEPROM				; seivi
				incf	EEADR,F					; järgmine aadress (ID-d ei kirjuta!)
				incf	EEADR,F					; järgmine aadress
				movff	Register272+.1,WREG		; PU-d - ANA pordile
				call	Wr_EEPROM	
				movff	Register272+.0,WREG		; PU-d DO - pordile
				call	Wr_EEPROM
				call	setup_serial			; seriali seaded kohe serial porti
				movff	Register273+.1,WREG		; seriali seaded uuesti sest kui oli viga, parandab pordi rutiin selle ära
				call	Wr_EEPROM				
				movff	Register275+.1,WREG		; IOD ehk ANA-pordi suund
				call	Wr_EEPROM				; kirjutame EEPROMi aga arvestame alles peale ANCONxi sättimist !
				movff	Register275+.0,WREG		; pordi Ana/Digi omadused (1=digi)
				call	Wr_EEPROM				; seivib EEPROMi
; *** resetid ****
				movlw	LOW(e_reset1)			; reset 1 ajad
				banksel	EEADR
				movwf	EEADR
				clrf	EEADRH
				movff	Register276,WREG		; side kadumise viiteaeg
				movwf	reset1dlytmrH
				call	Wr_EEPROM
				movff	Register276+.1,WREG		
				movwf	reset1dlytmrL
				call	Wr_EEPROM
				movff	Register277,WREG	
				movwf	reset1pulsetmrH			; reseti pulsi kestus
				call	Wr_EEPROM
				movff	Register277+.1,WREG			
				movwf	reset1pulsetmrL
				call	Wr_EEPROM
				movlw	LOW(e_reset2)			; reset 2 ajad
				movwf	EEADR
				clrf	EEADRH
				movff	Register278,WREG		; side kadumise viiteaeg
				movwf	reset2dlytmrH
				call	Wr_EEPROM
				movff	Register278+.1,WREG		
				movwf	reset2dlytmrL
				call	Wr_EEPROM
				movff	Register279,WREG	
				movwf	reset2pulsetmrH			; reseti pulsi kestus
				call	Wr_EEPROM
				movff	Register279+.1,WREG			
				movwf	reset2pulsetmrL
				call	Wr_EEPROM
; *** XORimine ****
				movff	Register271+.0,WREG		; DIGI-pordi väljundi aktiivse nivoo juhtimine
				call	Wr_EEPROM		
				movff	Register271+.1,WREG		; ANA-pordi väljundi aktiivse nivoo juhtimine
				call	Wr_EEPROM	
; *** ADC ****
				movff	Register270+.1,WREG		; ADC konf
				andlw	0x30	
				call	Wr_EEPROM
				movff	Register270+.1,WREG		; ADC konf uuesti
				andlw	0x30	
				BANKSEL ADCON1 
				MOVWF 	ADCON1 					; konfime kohe ADC ka ära
				banksel	.0
				movff	Register270+.0,WREG		; PWMi konf
				call	Wr_EEPROM
;				bcf		pwmenabled
;				btfsc	Register270,.0
;				bsf		pwmenabled
;				bcf		INTCON,TMR0IE
;				btfsc	pwmenabled
				bsf		INTCON,TMR0IE
; *** PWM ***
				movff	Register150+.0,WREG		; PWMi perioodi register
				call	Wr_EEPROM
				movff	Register150+.1,WREG
				call	Wr_EEPROM				
				movff	Register151+.0,WREG
				call	Wr_EEPROM				; PWMi konfi register
				movff	Register151+.1,WREG
				call	Wr_EEPROM				
; *** Dallase adr. lukustamine ***
				movlw	LOW(e_reg699)			; loe reg. 699-st lukustamise luba või keeld
				banksel	EEADR
				movwf	EEADR
				clrf	EEADRH
				movff	Register699+.0,WREG
				call	Wr_EEPROM				
				movff	Register699+.1,WREG
				call	Wr_EEPROM				
;===============================================================================
; kombineerib DO ja UIO juhtimise ja vastavate registrite sisud
;===============================================================================
setup_port:		movf	Register275+.1,W		; IOD ehk ANA-pordi suund. 1 = OUT
				andwf	ANCON0,F				; kõik sisendid tuleb teha digiks sest analoogsisend annab alati 0.
				comf	Register275+.1,W		; prosel on 1 = IN aga kood tahab et olex vastupidi
				movwf	datatemp				; häälestab ANA-pordi suuna ringi IOD järgi: ana-port on pordis A ja E !
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
				swapf	Register275+.1,W		; analoogpordi suund, 1= väljund, 0= sisend
				movwf	datatemp
				comf	datatemp,W
				movwf	datatemp
				rrcf	datatemp,W
				andlw	0x07
				banksel	TRISE
				movwf	TRISE					; ja porti E
				banksel	.0
; pordi Ana/Digi omadused. 1 = DIGI, 0 = ANA
				comf	Register275+.0,W		; 1=ANA: väljund ei saa olla analoog !
				movwf	Registertemp7
				comf	Register275+.1,W		; 0 = OUT
				andwf	Registertemp7,W
				banksel	ANCON0
				movwf	ANCON0					; prose ANCONx registrisse
				clrf	ANCON1
				banksel	.0

				movf	Register275+.0,W		; analoogpinnile vastav bitt registris 1+1 (UIO sisendid) => 0
				banksel	Register1
				andwf	Register1+.1,F
				banksel .0

				movff	Register0+.1,WREG		; taastame UIO väljundite seisu vastava registri järgi
				andwf	Register275+.0,W		; maskeerime ANA/digi oamdusega (digi=1)
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
				incf	EEADR,F					; järgmine aadress
				movff	EEDATA,WREG				; meenutame datat
				return
;===============================================================================
;  EEPROMist lugemine
;===============================================================================
Read_EEPROM:	BCF 	EECON1,EEPGD			; Point to DATA memory
				bsf		EECON1,RD			 	; loe!
				movff	EEDATA,WREG
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
 			  clrf	    DS1820found	 ; esialgu pole ühtegi Dallase mölakat leitud ;)
			  clrf		DS2438found
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
			  clrwdt
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
              rlcf       bits,F          		; shift bit A over to the left
              call      Readbit_1wire    		; read bit B from bus and 
              iorwf     bits,F			        ; save it

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
              movf      TEMP0,w

lookup_2x:	  addlw		.0
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
			  clrwdt
; Send rombit(rombit_idx) to 1-wire bus
ack_bus:	  clrwdt
			  bcf		CARRY
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
              	rrcf     OneWireByte,F   		; Load CARRY with bit to send
              	btfsc   CARRY			        ; See what we are sending
               	bsf     TEMP1,0         		; must be a 1
              	movf    TEMP1,W		            ; Load up the bit and
              	call    Sendbit_1wire    		; send it out
              	decfsz  TEMP0,F         		; 8 bitti ok ?
               	goto     _send_1w_lp      		; vara veel
             	return
;;===============================================================================
;; ******* saadab käsubiti I-nööbile *******
;;===============================================================================
;Sendbit_1wire:	btfsc	noints
;				goto	sb1w1
;				bcf	INTCON,GIE
;				bcf		INTCON,PEIE			
;sb1w1:			movwf   TEMP1
;
;              	bcf     _1WDAT	         		; madalax
;              	btfsc   TEMP1,.0 
;               	bsf     _1WDAT	        		; pulsi lõpp kui bitt oli 1
;
;;              	btfsc   TEMP1,.0 
;;				goto	sb1wH
;;              	bcf     _1WDAT	         		; saadame nulli
;;				goto	sb1w2
;;sb1wH:			bsf     _1WDAT	         		; saadame ühe
;
;sb1w2:			movlw	.58						; (Tslot + Trec) - Tlow1
;				call	wait_x
;	            bsf     _1WDAT        			; pulsi lõpp kui bitt oli 0
;              	nop    							; Trec
;				btfsc	noints
;				return
;				bsf		INTCON,PEIE			
;				bsf	INTCON,GIE
;	            return
;===============================================================================
; ******* saadab käsubiti I-nööbile *******
;===============================================================================
Sendbit_1wire:	btfsc	noints
				goto	sb1w1
				bcf	INTCON,GIE
				bcf		INTCON,PEIE			
sb1w1:			movwf   TEMP1
              	bcf     _1WDAT	         		; Write slot algab: madalax
				call	w_1us					; ootab min 1us
              	btfsc   TEMP1,.0 				; mida saata
				goto	sb1wH
              	bcf     _1WDAT	         		; saadame nulli
				goto	sb1w2
sb1wH:			call	w_1us					; ühe saatmisel venitame selguse mõttes veica
				call	w_1us					
				call	w_1us					
				bsf     _1WDAT	         		; saadame ühe

sb1w2:			movlw	.40;58					; Tslot=60us
				call	wait_x
	            bsf     _1WDAT        			; pulsi lõpp kui bitt oli 0
				call	w_1us					; recovery 1us
				btfsc	noints
				return
				bsf		INTCON,PEIE			
				bsf	INTCON,GIE
	            return
;===============================================================================
w_1us:			nop								; ootab min 1us
				nop
				nop
				nop
				nop
				nop
				nop
				nop
				nop
				nop
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
              	rrcf     OneWireByte, F   		; Rotate bit into IO area
	            decfsz  TEMP0,F		 	        ; 8 bitti OK?
               	goto     _read_1w_lp      		; vara veel
              	movf    OneWireByte,W		    ; jah, tulemus W-sse
             	return                     ;
;===============================================================================
; ******* loeb vastusbiti I-nööbist *******
;===============================================================================
Readbit_1wire:	btfsc	noints
				goto	rb1w1
				bcf	INTCON,GIE
				bcf		INTCON,PEIE			
rb1w1:;				bcf		TRIS_1WDAT
			bcf    	_1WDAT	         		; taktipulss - madalax
				call	w_1us					; ootab min 1us
	            bsf    	_1WDAT	         		; taktipulss - kõrgex
				bsf		TRIS_1WDAT
	            clrf    TEMP1            		; Assume incomming bit is 0
				movlw	.7;12					; oota 13uS
				call	wait_x
 	            btfsc   _1WDAT	         		; What are we receiving?
                bsf     TEMP1,0			        ; must be a 1
				movlw	.31;47					; oota 48uS
				call	wait_x
				bcf		TRIS_1WDAT
              	movf    TEMP1,W
				call	w_1us					; ootab min 1us
				btfsc	noints
				return
				bsf		INTCON,PEIE			
				bsf	INTCON,GIE
	            return
;;===============================================================================
;; ******* loeb vastusbiti I-nööbist *******
;;===============================================================================
;Readbit_1wire:	btfsc	noints
;				goto	rb1w1
;				bcf	INTCON,GIE
;				bcf		INTCON,PEIE			
;rb1w1:			bcf    	_1WDAT	         		; taktipulss - madalax
;				nop
;	            bsf    	_1WDAT	         		; taktipulss - kõrgex
;				bsf		TRIS_1WDAT
;	            clrf    TEMP1            		; Assume incomming bit is 0
;				movlw	.12						; oota 12uS
;				call	wait_x
; 	            btfsc   _1WDAT	         		; What are we receiving?
;                bsf     TEMP1,0			        ; must be a 1
;				movlw	.47						; oota 47uS
;				call	wait_x
;              	movf    TEMP1,W
;				bcf		TRIS_1WDAT
;				btfsc	noints
;				return
;				bsf		INTCON,PEIE			
;				bsf	INTCON,GIE
;	            return
;===============================================================================
; Store_Bit võtab bitt A (bits.1) ja salvestab alasse work, offset rombit_idx
; bits.1 -> work0..work7(rombit_idx)
;===============================================================================
Store_Bit:    call      SetupFSR         		; converdi rombit_idx mälu aadressiks
              rrcf       bits, W          		; loe bit.1 value right justified into W
              andlw     b'00000001'     		; lolle aadresse ei luba
			  btfsc		ZERO
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
decrtmrs:
T1intr1end:		btfsc	reset1pulseon			; reseti 1 pulss juba aktiivne ?
				goto	T1intr1end1				; jah
				movf	reset1dlytmrH,W			; kas side viiteaeg anti = 0 ?
				addlw	.0
				btfss	ZERO
				goto	sv9						; eip
				movf	reset1dlytmrL,W
				addlw	.0
				btfss	ZERO
				goto	sv9						; eip
				goto	R1zero; sv10					; jah, siis on aeg oodatud !
sv9:			decfsz	reset1dlytmrL			; ootame kannatlikult, ehk side taastub
				goto	T1intr2					
				movf	reset1dlytmrH,W			; HIGH juba on  ?
				addlw	.0
				btfsc	ZERO
				goto	sv10					; jah, siis aeg täis
				decf	reset1dlytmrH,F
				decf	reset1dlytmrL,F
				goto	T1intr2
sv10:			movff	Register277,WREG		; piisavalt InterNetita oldud: 
				movwf	reset1pulsetmrH			; laeme pulsi kestuse
				addlw	.0
				btfss	ZERO					; nulline kestus tähendab, et pulssi ei tekitatagi !
				goto	R1notzero
				movff	Register277+.1,WREG	
				movwf	reset1pulsetmrL		
				addlw	.0
				btfsc	ZERO					; nulline kestus tähendab, et pulssi ei tekitatagi !
				goto	R1zero
R1notzero:		bsf		reset1pulseon			; märgime ära
				bcf		n_reset1				; ja nüüd annab saapaga ... ! -> inversioonis
R1zero:			movff	Register276,WREG		; taasta side kadumise viiteaeg
				movwf	reset1dlytmrH
				movff	Register276+.1,WREG
				movwf	reset1dlytmrL
				goto	T1intr2					
T1intr1end1:	decfsz	reset1pulsetmrL			; tixub pulsi kestust
				goto	T1intr2					
				movf	reset1pulsetmrH,W		; HIGH juba on  ?
				addlw	.0
				btfsc	ZERO
				goto	sv11					; jah, siis aeg täis
				decf	reset1pulsetmrH,F
				decf	reset1pulsetmrL,F
				goto	T1intr2
sv11:			bsf		n_reset1				; aitab kah pexmisest... -> inversioonis
				bcf		reset1pulseon			; märgime ära
				movff	Register277,WREG		; laeme uuesti pulsi kestuse
				movwf	reset1pulsetmrH
				movff	Register277+.1,WREG
				movwf	reset1pulsetmrL
;***********************
T1intr2:		btfsc	reset2pulseon			; reseti 2 pulss juba aktiivne ?
				goto	T2intr1end1				; jah
				movf	reset2dlytmrH,W			; kas side viiteaeg anti = 0 ?
				addlw	.0
				btfss	ZERO
				goto	sv29					; eip
				movf	reset2dlytmrL,W
				addlw	.0
				btfss	ZERO
				goto	sv29					; eip
				goto	R2zero;sv20					; jah, siis on aeg oodatud !
sv29:			decfsz	reset2dlytmrL			; ootame kannatlikult, ehk side taastub
				return					
				movf	reset2dlytmrH,W			; HIGH juba on  ?
				addlw	.0
				btfsc	ZERO
				goto	sv20					; jah, siis aeg täis
				decf	reset2dlytmrH,F
				decf	reset2dlytmrL,F
				return
sv20:			movff	Register279,WREG		; piisavalt InterNetita oldud: 
				movwf	reset2pulsetmrH			; laeme pulsi kestuse
				addlw	.0
				btfss	ZERO					; nulline kestus tähendab, et pulssi ei tekitatagi !
				goto	R2notzero
				movff	Register279+.1,WREG	
				movwf	reset2pulsetmrL		
				addlw	.0
				btfsc	ZERO					; nulline kestus tähendab, et pulssi ei tekitatagi !
				goto	R2zero
R2notzero:		bsf		reset2pulseon			; märgime ära
				bcf		PWRSW					; ja nüüd annab saapaga ... ! -> inversioonis
R2zero:			movff	Register278,WREG		; taasta side kadumise viiteaeg
				movwf	reset2dlytmrH
				movff	Register278+.1,WREG	
				movwf	reset2dlytmrL
				return					
T2intr1end1:	decfsz	reset2pulsetmrL			; tixub pulsi kestust
				return					
				movf	reset2pulsetmrH,W		; HIGH juba on  ?
				addlw	.0
				btfsc	ZERO
				goto	sv21					; jah, siis aeg täis
				decf	reset2pulsetmrH,F
				decf	reset2pulsetmrL,F
				return
sv21:			bsf		PWRSW					; aitab kah pexmisest... -> inversioonis
				bcf		reset2pulseon			; märgime ära
				movff	Register279,WREG		; laeme uuesti pulsi kestuse
				movwf	reset2pulsetmrH
				movff	Register279+.1,WREG	
				movwf	reset2pulsetmrL
				return
;===============================================================================
; ******************* DSxxxx funktsioonid **************************************
;===============================================================================
;===============================================================================
; ******* loeb andurilt näidu ja käivitab järgmise mõõtmise *******
;===============================================================================
GetTemp:		decfsz	_10sekunditmr			; tegelikult 1sekund !
				goto	GetTemp_end
				movlw	_10sekundiaeg
				movwf	_10sekunditmr
				movf    DS1820found,W			; kas on üldse termoandureid ?
				addlw	.0
				btfss	ZERO
				goto	gettemp1				; on !
				movf    DS2438found,W			; aga neid teisi kive ?
				btfsc	ZERO
				goto	GetTemp_end				; ei ole midagi...
gettemp1:		bsf		temper
				goto	GetTemp_end				; mõõdab siis kui aega on



GetT:			bsf		_1WSPU					; tugev toide maha 
				movlw	.16						; kui >16 siis kusitleme DS2438 kive
				cpfslt	DallasState
				goto	GetDS2438				; nii ongi !
				call	Reset_1wire				; liinile saabast
				call	saada_adr				; saada anduri aadress
				movlw	ReadScratch				; loe tulemust
				call	Sendbyte_1wireA
				decf	DallasState,W			; kalkuleerib anduri näidu aadressi
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

				call	chk_temp				; kas 85C (või ei vastatud - kui MSB = 0xFF) ?
				btfss	CARRY
				goto	temp_nxt0				; ei ole 85C, kas vastus puudus üldse ?
gett1:			btfsc	INDF1,.0				; esimene kord ?
				goto	Get_t_sec				; ei, teine kord
				movlw	0x03					; märgib, et oli paha näit ja nüüd on 2. kord
				movwf	INDF1
;				goto	Get_temp_dn				; ja seda näitu ei usu
				call	Start_temp				; seda näitu ei usu, käivita uus mõõtmine
				bcf		_1WSPU					; tugev toide peale
				goto	GetTemp_end				; mõõdame uuesti !
Get_t_sec:		btfsc	INDF1,.1				; 2. kord: kas eelmine oli samuti 85C ?
				goto	temp_nxt00				; jah aga kas äkki vastus puudub üldse?
gett2:			clrf	INDF1					; eelmine näit oli midagi muud, ei usu! markerid maha 
				movlw	0x10					; kirjutame näiduks 4096 (0x1000)
				movwf	POSTINC0
				clrf	POSTINC0				
				goto	Get_temp_dn
temp_nxt0:		btfss	ZERO
				goto	temp_nxt1				; kõik oli OK
				goto	gett1					; vastus puudus oopistykkis

temp_nxt00:		btfss	ZERO
				goto	temp_nxt1				; jah, siis usaldame
				goto	gett2					; vastus puudus, märgi ära!
temp_nxt1:		clrf	INDF1					; markerid maha
;
; siia võrdlus: kui >7d0 või F828 siis näitu ei uuendata
temp_nxt:		movf	work0,W
				sublw	0x07
				btfss	CARRY
				goto	Get_temp_dn
				movf	work0,W
;
				movwf	POSTINC0
				movff	work7,POSTINC0			; meenutame LSB-d
Get_temp_dn:	call	Start_temp				; käivita uus mõõtmine
				bcf		_1WSPU					; tugev toide peale 
				decf	DallasState,F			; 1 andur läbi käidud
				movf    DallasState,W			; kõik ?
				addlw	.0
				btfss	ZERO
				goto	GetTemp_end				; eip !
				movf    DS2438found,W			; kas neid teisi kive ka on ?
				btfsc	ZERO
				goto	Get_temp_dn1			; ei - loeme vaid DS1820-d
				swapf    DS2438found,W			; DS2438 kivide hulk läheb vanemaks nibleks
				addlw	.4						; noorem näitab suhtlemise seisundimasina seisu. Igale kivile on 4 seisundit
				movwf	DallasState
				goto	GetTemp_end				; selguse mõttes...

Get_temp_dn1:	movf    DS1820found,W			; alustame algusest
				andlw	0x0F					; tarbetu...
				movwf	DallasState
GetTemp_end:	return

chk_temp:		movf	work0,W					; Cy=1 kui näit 0x550 ehk 85 C. Kui Z=1 siis vastus puudus
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
				bsf		ZERO					; märgime et ei vastatud
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

; 4 käivita T
; 3 loe T, käivita Vdd
; 2 loe Vdd, lülita Vadc, käivita Vadc
; 1 loe Vadc, ja I
GetDS2438:		movf	DallasState,W			; mida teha ?
				andlw	0x0F
				btfsc	ZERO
				goto	GetDS2438_nxt			; võtta järgmine andur
				sublw	.1
				btfsc	ZERO
				goto	GetDS2438_s1			; state 1 ehk loe Vadc ja I ning mine seisu 0
				movf	DallasState,W
				andlw	0x0F
				sublw	.2
				btfsc	ZERO
				goto	GetDS2438_s2			; state 2 ehk loe Vdd ja lülita Vadc külge ja käivita Vadc
				movf	DallasState,W
				andlw	0x0F
				sublw	.3
				btfsc	ZERO
				goto	GetDS2438_s3			; state 3 ehk loe T ja lülita Vdd külge ja käivita Vdd
GetDS2438_s4:	call	Reset_1wire				; state 4 ehk käivita T mõõtmine. Liinile saabast
				call	saada_adr				; saada anduri aadress
				movlw	ConvertT				
				call	Sendbyte_1wireA
				bcf		_1WSPU					; tugev toide peale
				decf	DallasState,F			; võtab järgmise seisu
;
				movlw	T1resoL					; lae taimer 1 uuesti
				movwf	TMR1L
				movlw	T1resoH
				movwf	TMR1H

				return
GetDS2438_s3:	call	Reset_1wire				; seis 3: loe T, käivita Vdd
				call	saada_adr				; saada anduri aadress
				movlw	RecallMem				; kopeeri tulemused scratchpad'i
				call	Sendbyte_1wireA
				movlw	.0						; lk. 0
				call	Sendbyte_1wireA
				call	Reset_1wire				; saabast ja siis räägib edasi
				call	saada_adr				; saada anduri aadress
				movlw	ReadScratch				; loe tulemust
				call	Sendbyte_1wireA
				movlw	.0						; lehekülg 0
				call	Sendbyte_1wireA
				swapf	DallasState,W			; kalkuleerib anduri näidu aadressi
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
;--- temperatuuri puhul kontrollime kas vastus oli ja kui puudus, kirjutab näidux 0x1000 ---
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
				goto	dtemp_nxt				; kõik oki-doki
dgett1:			btfsc	INDF1,.0				; esimene kord ?
				goto	dGet_t_sec				; ei, teine kord. Nüüd küll lajatab...
				movlw	0x01					; märgib, et oli paha näit ja nüüd on 2. kord
				movwf	INDF1
				goto	dGet_temp_dn			; ja seda näitu ei usu (aga veel üle ei kirjuta !)
dGet_t_sec:		clrf	INDF1					; ei vastata: markerid maha 
				movlw	0x10					; ja kirjutame näiduks 4096 (0x1000)
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
				movff	work0,POSTINC0			; salvesta näit: MSB
				movff	work7,POSTINC0			; ja LSB
;--- temperatuuri puhul kontrollime kas vastus oli ja kui puudus, kirjutab näidux 0x1000 ---
;				call	Readbyte_1wire			; loe staatus, unusta
;				call	Readbyte_1wire			; loe temperatuur (LSB)
;				movwf	work7					; seivi sest tahan hiljem saata  MSB esimesena
;				call	Readbyte_1wire			; MSB
;				movwf	POSTINC0
;				movff	work7,POSTINC0			; LSB
dGet_temp_dn:	call	Reset_1wire				; lülitame ADC Vdd külge
				call	saada_adr				; saada anduri aadress
				movlw	WrScr					; kirjuta scratchpad'i
				call	Sendbyte_1wireA
				movlw	.0						; lk. 0
				call	Sendbyte_1wireA
				movlw	_VddConnected			; lülita ADC sisend Vdd külge
				call	Sendbyte_1wireA
				call	Reset_1wire				; stardime pinge mõõtmise
				call	saada_adr				; saada anduri aadress
				movlw	ConvertV				
				call	Sendbyte_1wireA
				bcf		_1WSPU					; tugev toide peale
				decf	DallasState,F			; võtab järgmise seisu	
				return
GetDS2438_s2:	call	Reset_1wire				; seis 3: loe Vdd, lülita Vadc, käivita Vadc
				call	saada_adr				; saada anduri aadress
				movlw	RecallMem				; kopeeri tulemused scratchpad'i
				call	Sendbyte_1wireA
				movlw	.0						; lk. 0
				call	Sendbyte_1wireA
				call	Reset_1wire				; saabast ja siis räägib edasi
				call	saada_adr				; saada anduri aadress
				movlw	ReadScratch				; loe tulemust
				call	Sendbyte_1wireA
				movlw	.0						; lehekülg 0
				call	Sendbyte_1wireA
				swapf	DallasState,W			; kalkuleerib anduri näidu aadressi
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
				call	Reset_1wire				; lülitame ADC Vadc külge
				call	saada_adr				; saada anduri aadress
				movlw	WrScr					; kirjuta scratchpad'i
				call	Sendbyte_1wireA
				movlw	.0						; lk. 0
				call	Sendbyte_1wireA
				movlw	_VadcConnected			; lülita ADC sisend Vadc külge
				call	Sendbyte_1wireA
				call	Reset_1wire				; stardime pinge mõõtmise
				call	saada_adr				; saada anduri aadress
				movlw	ConvertV				
				call	Sendbyte_1wireA
				bcf		_1WSPU					; tugev toide peale
				decf	DallasState,F			; võtab järgmise seisu	
				return
GetDS2438_s1:	call	Reset_1wire				; seis 1: loe Vadc, ja I
				call	saada_adr				; saada anduri aadress
				movlw	RecallMem				; kopeeri tulemused scratchpad'i
				call	Sendbyte_1wireA
				movlw	.0						; lk. 0
				call	Sendbyte_1wireA
				call	Reset_1wire				; saabast ja siis räägib edasi
				call	saada_adr				; saada anduri aadress
				movlw	ReadScratch				; loe tulemust
				call	Sendbyte_1wireA
				movlw	.0						; lehekülg 0
				call	Sendbyte_1wireA
				swapf	DallasState,W			; kalkuleerib anduri näidu aadressi
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
				decf	DallasState,F			; võtab järgmise seisu. Peaks olema 0 ehk alustame algusest
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
				addlw	.4						; alustame jälle seisundist 4
				movwf	DallasState
				bcf		_1WSPU					; tugev toide peale 
				return
Dallas_algusest:movf	DS1820found,W			; alustame viimase DS1820-ga
				andlw	0x0F					; kas DS1820-d on üldse ?
				addlw	.0
				btfss	ZERO
				goto	da1						; on ikka
				swapf	DS2438found,W			; eip, võtame DS2438-d
				andlw	0xF0
				addlw	.4
				movwf	DallasState
				bcf		_1WSPU					; tugev toide peale 
				return
da1:			movwf	DallasState
				call	Start_temp				; käivita uus temp. mõõtmine
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
		movlw	MatchRom				; käsk vaid konkreetsele andurile
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
				goto	saada_adr_loop			; saadab kõik ID 8 baiti
				return
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
; ******* viited *******
;===============================================================================
wait:			movlw	0xFF
wait_x:	;		bsf		pank
				movwf	PR2
;				bcf		pank
;			movlw	0x01					; T2 periood 1uS, seisma!
			movlw	0x02					; T2 periood 1uS, seisma! - > nüüd 1:16 prescaler (oli 1:4)
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
init_a1:		movlw	mõõtmisi
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
				clrf	INTCON
				clrf	WDTCON
				clrwdt                    		; WDT nullida 
; **** konf ****
				bsf		INTCON2,.7				; B-pullupid maha
;*** Taimer 0 *****
;				movlw	B'01000011'				; 8 bitine, 1:16 prescaler T0-le, taimerina, seisma !
				movlw	B'01000101'				; 8 bitine, 1:64 prescaler T0-le, taimerina, seisma !
				movwf	T0CON					
				movlw	T0reso					; lae taimer 0 (katkestus iga 250 us tagant)
				movwf	TMR0L
; **** komparaatorid ****
				BANKSEL CM1CON 
				movlw 	0x00					; analoog komparaatorid välja
				movwf 	CM1CON 
				movwf 	CM2CON 
				clrf	CVRCON					; tugipinge allikas OFF
;--- A/D muundi ---
				BANKSEL ADCON1 
				MOVLW 	B'00110000' 			; Selects the special trigger from the ECCP1,Vref=4,1V,A/D VREF-=0,Analog Negative Channel=GND
				MOVWF 	ADCON1 					
				BANKSEL ADCON2 
;				MOVLW 	B'10111010' 			; right justified,-, 20 TAD, FOSC/32
				MOVLW 	B'10111110' 			; right justified,-, 20 TAD, FOSC/64
				MOVWF 	ADCON2 					
				BANKSEL ADCON0 
				MOVLW 	chan1		 			; kanal 1
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
				movlw	0xC4					; serial ja RdRxD on sisend, DIR ja muud väljunditeks (v.a. 1Wire data otc ja C3 mis oli enne reset2)
				movwf 	TRISC 
				banksel PORTC    		 
				movlw	0xCF;8;7					; port sellisesse lähteseisu (n_reset1 teisipidi) Viimane asi - C0 ehk 1WSPU püsti
				movwf 	PORTC 
				banksel TRISD     		 
				movlw	0x00					; kõik väljunditeks
				movwf 	TRISD 
				banksel PORTD    		 
				movlw	0x00					; port sellisesse lähteseisu
				movwf 	PORTD 
				movlw	0x08					; port sellisesse lähteseisu
				movwf 	PORTE 
;********* Vahipeni ketti **************
				clrwdt                    		; WDT nullida 
				call	ajupesu					; mälu killimine
;>>> CHG
				banksel	EECON1
				clrf	EECON1
				banksel	.0
;>>> CHG
				call	Read_Setup				; analoogkanali portide setup jms				
;--- USART -------------------- 				; USARTi häälestus (SYNC = 0, BRGH = 0, BRG16 = 1)
; Register 273
; b0,1,2 - kiirus, default 19200 (010)
; b3 -debounce kõigile sisenditele, default  ON
; b4 - sticky bitt kõigile sisenditele, default OFF
; b5,6 - Wiegand B,A lubatus
; b7 - paarsuse bitt, (0= even parity), default EVEN
				call	setup_serial
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
				movlw	0xFF					; int peale 256 uS
				movwf	PR2
				bcf		PIE1,TMR2IE
				clrf	TMR2
				bcf		PIR1,TMR2IF
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
				movlw	B'00010000'				; katskestusi ei ole esinenud, RS232 porti ei puudu: read-only
				movwf	PIR1	
				banksel	IOCB
				movlw	0xFF
				movwf	IOCB					; lubame kõik pordi B muutuse katckestused
				banksel	.0
				bcf		INTCON,PEIE	
				bsf		CCPTMRS,C2TSEL		
;--- Variaablid ----------------				; muu pudi
				movf	DinPort,W
				movwf	dinpress
				comf	DinPort,W
	movf	DinPort,W
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
				movwf	lastinputs1				; anapordi sisendite eelmine seis oli 0xFF			
				movwf	lastinputs2		
				movlw	_10sekundiaeg
				movwf	_10sekunditmr
				movlw	.4						; 1ms loendi
				movwf	_1mscount

				LFSR	.1,Dallas1				; DS1820 kontrollbaidid näidaku et andurid puuduvad (0x1000)
				movlw	.9
				movwf	DS1820found
ds1820loop:		movlw	0x10
				movwf	POSTINC1
				movlw	0x00
				movwf	POSTINC1
				decfsz	DS1820found
				goto	ds1820loop

				LFSR	.1,DS2438_1				; DS2438 kontrollbaidid näidaku samuti et andurid puuduvad (0x1000)
				movlw	.9*.4
				movwf	DS1820found
ds2438loop:		movlw	0x10
				movwf	POSTINC1
				movlw	0x00
				movwf	POSTINC1
				decfsz	DS1820found
				goto	ds2438loop
				movff	Register699+.1,WREG		; kas detectida või lugeda eepromist?
				sublw	.1
				btfss	ZERO
				goto	discovery				; vaatame ise järgi
				goto	load_dallas_adr			; lae aadressid eepromist

;--- Dallase andurite avastamine ----------------
discovery:		clrwdt
				call    Search_1Wire_init		; Dallase lugemine nulli
				bcf		nomoreds2438			; DS2438-d pole veel piisavalt leitud
				bcf		temper					; DS1820-d pole veel piisavalt leitud
				clrf	DS1820found
				clrf	DS2438found
				clrf	DallasState
				clrwdt                    		; WDT nullida 
				bsf		noints
;	      goto    _search_end     		; DEBUG 1W andureid ei ole
_search_next:	call    Search_1Wire     		; käse otsida              
				clrwdt                    		; WDT nullida 
;	movf	return_value,W
;	bsf Dir
;	call	SendCHAR
;	bcf Dir

            	btfss   return_value,0  		; Oli midagi ?
	            goto    _search_end     		; sitte mittagi...
				bcf		dallas					; kui dallas=1 siis DS2438
				LFSR	.1,work7				; 8. bait on family code. DS18B20=0x28, DS2438=0x26. Selle järgi määrame mäluaadressi kuhu ID salvestada
				movf	INDF1,W					; väidab, et leidis miskit aga kui liin oli lühises ?
				addlw	.0
				btfsc	ZERO
	            goto    _search_end     		; perset, Roosi, oligi lühis !
				movf	INDF1,W
				sublw	DS1820ID
				btfss	ZERO
				bsf		dallas					; oli DS2438 !
				btfsc	dallas					; kas neid andureid on juba piisavalt ?
				goto	disc_2
				btfsc	temper					; kas DS1820-d juba küllalt ?
				goto	disc_end				; jah, jäta vahele
				goto	disc_3					; veel on ruumi
disc_2:			btfsc	nomoreds2438			; kas DS2438-d juba sitaks palju ?
				goto	disc_end				; jah, jäta vahele
				
disc_3:			LFSR	.0,Dallas1wa			; määrame õige mäluala alguse
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
				btfss	dallas					; loendame õigeid andureid
				goto	dl_mv_1
				incf	DS2438found,F			; DS2438 juures
				movlw    MaxDS2438Sensors
				subwf    DS2438found,W
				btfss    ZERO			        ; kas piir käes ?
				goto	_search_next			; eip, võta nekst Dallase mölakas
				bsf		nomoreds2438			; DS2438-d on leitud max kogus, rohkem enam ei luba
				goto	_search_next			; piisavalt aga otsime ikkagi lõpuni

dl_mv_1:		incf	DS1820found,F			; loendame andureid
				movlw    MaxDS1820Sensors
				subwf    DS1820found,W
				btfss    ZERO			        ; kas piir käes ?
				goto	_search_next			; eip, võta nekst Dallase mölakas
				bsf		temper					; DS1820-d on leitud max kogus, rohkem enam ei luba
				goto	_search_next			; piisavalt aga otsime ikkagi lõpuni

disc_end:		btfss	temper					; DS1820-e arv täis ?
				goto	_search_next			; eip, otci veel
				btfss	nomoreds2438			; aga DS2438 ?
				goto	_search_next			; eip, otci veel
disc_end1:		call	Start_temp				; kõik andurid mõõtma ja kohe!
				bcf		_1WSPU					; tugev toide peale 
				movf    DS1820found,W	
				andlw	0x0F					; tarbetu küll aga las olla...		
				movwf	DallasState
				movlw	sekundiaeg
				movwf	sekunditmr
				goto	_search_done			; lõpetame jama...
_search_end:	movf    DS1820found,W			; enam ei leia kuid kas enne leiti miskit ?
				addlw	.0
				btfss	ZERO
				goto	disc_end1				; midagi leiti, stardime temp. mõõtmised
				movf    DS2438found,W			; aga DS2438 ?
				addlw	.0
				btfsc	ZERO
				goto	_search_done			; midagi ei leitud
				bcf		_1WSPU					; tugev toide peale 
				swapf    DS2438found,W			; leidsime vaid DS2438 kivid
				andlw	0xF0					; tarbetu küll aga las olla...
				addlw	.4						; alustame seisust 4		
				movwf	DallasState
				movlw	sekundiaeg
				movwf	sekunditmr
_search_done:	clrwdt                    		; WDT nullida 
				call	save_dallas_adr			; salvesta leitu eepromi
				clrwdt
_search_done1:	clrf	Measflags
			bcf		noints
				call	reset_ser1				; sidetaimeri reload
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
;*************** Laeb 1-wire aadressid eepromist *******************************
;===============================================================================
load_dallas_adr:bsf		noints
				movlw	.72
				movwf	countL
				LFSR	.0,Dallas1wa
				movlw	LOW(e_ds1820_1)			; loe DS1820 aadressid
				banksel	EEADR
				movwf	EEADR
				clrf	EEADRH
load_dallas_1:	call	Read_EEPROM					
				movff	WREG,POSTINC0
				decfsz	countL
				goto	load_dallas_1
				call	Read_EEPROM				; DS1820 hulk			
				movff	WREG,DS1820found
				movlw	.72
				movwf	countL
				LFSR	.0,DS24381wa
				movlw	LOW(e_ds2438_1)			; loe DS2438 aadressid
				banksel	EEADR
				movwf	EEADR
				clrf	EEADRH
load_dallas_2:	call	Read_EEPROM					
				movff	WREG,POSTINC0
				decfsz	countL
				goto	load_dallas_2
				call	Read_EEPROM				; DS1820 hulk			
				movff	WREG,DS2438found
				goto	load_d1

load_dallas_3:	call	Start_temp				; kõik andurid mõõtma ja kohe!
				bcf		_1WSPU					; tugev toide peale 
				movf    DS1820found,W	
				andlw	0x0F					; tarbetu küll aga las olla...		
				movwf	DallasState
				movlw	sekundiaeg
				movwf	sekunditmr
				goto	load_dallas_4			; lõpetame jama...

load_d1:		call    Reset_1wire
				movf    DS1820found,W			; enam ei leia kuid kas enne leiti miskit ?
				addlw	.0
				btfss	ZERO
				goto	load_dallas_3			; midagi leiti, stardime temp. mõõtmised
				movf    DS2438found,W			; aga DS2438 ?
				addlw	.0
				btfsc	ZERO
				goto	load_dallas_4			; midagi ei leitud
				bcf		_1WSPU					; tugev toide peale 
				swapf    DS2438found,W			; leidsime vaid DS2438 kivid
				andlw	0xF0					; tarbetu küll aga las olla...
				addlw	.4						; alustame seisust 4		
				movwf	DallasState
				movlw	sekundiaeg
				movwf	sekunditmr
load_dallas_4:	clrwdt                    		; WDT nullida 
			bcf		nomoreds2438			
			bcf		temper					
				goto	_search_done1
;===============================================================================
;*************** Salvestab 1-wire aadressid eepromi ****************************
;===============================================================================
save_dallas_adr:movlw	.72
				movwf	countL
				LFSR	.0,Dallas1wa
				movlw	LOW(e_ds1820_1)			; loe DS1820 aadressid
				banksel	EEADR
				movwf	EEADR
				clrf	EEADRH
save_dallas_1:	movff	POSTINC0,WREG
				call	Wr_EEPROM					
				decfsz	countL
				goto	save_dallas_1
				movff	DS1820found,WREG		; DS1820 hulk
				call	Wr_EEPROM					
				movlw	.72
				movwf	countL
				LFSR	.0,DS24381wa
				movlw	LOW(e_ds2438_1)			; loe DS2438 aadressid
				banksel	EEADR
				movwf	EEADR
				clrf	EEADRH
save_dallas_2:	movff	POSTINC0,WREG
				call	Wr_EEPROM					
				decfsz	countL
				goto	save_dallas_2
				movff	DS2438found,WREG		; DS2438 hulk
				call	Wr_EEPROM					
				return
;===============================================================================
;************************ Bootloaderi koodijupid *******************************
;===============================================================================
				org	0xEA00
ErrorLoader:
				org	0xF000
BootLoader:
BootInts:	    movwf   W_Temp           		; Seivi kontekst        
				swapf   STATUS,W         
				movwf   S_Temp   
				movff	FSR0L,FSRtmpL
				movff	FSR0H,FSRtmpH
				banksel	.0
 				btfsc	PIR1,TMR1IF				; Taimer 1'e katckestus?
				call	bT1int
				btfsc	PIR1,RC1IF				; Seriali katckestus?
				call	bSerInt					
				movff	FSRtmpL,FSR0L
				movff	FSRtmpH,FSR0H
				swapf   S_Temp,W         
				movwf   STATUS           		; taasta STATUS         
				swapf   W_Temp,F         
				swapf   W_Temp,W         		; Taasta W         
				retfie                   	
;===============================================================================
;*********************** Taimer 1'e INT ****************************************
;===============================================================================
; * T1 on süsteemi taimer intervalliga 10 ms.
bT1int:			bcf     PIR1,TMR1IF    			; katkestuse nõue maha 
				movlw	T1resoL					; lae taimer 1 uuesti
				movwf	TMR1L
				movlw	T1resoH
				movwf	TMR1H
;********* Vahipeni ketti **************
				clrwdt                    		; WDT nullida 
;**** sekundi aja taimerid ****
				decfsz	sekunditmr
				goto	bT1int_1
				movlw	sekundiaeg
				movwf	sekunditmr
;**** sidepaketi taimer ****
bT1int_1:		btfss	SerialTimerOn			; seriali taimer käib?
				goto	bT1int_2
				decfsz	serialtimer				; aeg täis?
				goto	bT1int_2
				call	breset_ser
				bcf		PIR1,RC1IF				; katkestuse nõue maha
bT1int_2:
				return
;===============================================================================
; ******* Seriali INT *******
;===============================================================================
bSerInt:	
;********* Vahipeni ketti **************
				clrwdt                    		; WDT nullida 
				bcf		PIR1,RC1IF				; katkestuse nõue maha
				banksel	RCSTA1
				movf	RCSTA1,W				; oli viga?
				banksel	.0
				andlw	.6						; Viga vastuvõtul? Maskeeri tarbetud staatuse bitid
				btfss	ZERO
				goto	breset_ser2				; oli, alusta uuesti
bmodbus_rcv:		movf	bytecnt,W				; kontrolli puhvri piire
				sublw	RSBufLen-.1
				btfss	CARRY
				goto	breset_ser2				; liiga pikk mess või miskit sassis, reset!
				bsf		SerialTimerOn	
				banksel	RCREG1
				movf	RCREG1,W
				banksel .0
				movwf	Char
				LFSR	.0,Puhver				; arvutame baidi salvestamise koha vastuvõtupuhvris
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
				movf	bytecnt,W				; äkki oli teine (käsk) ?
				sublw	.2
				btfss	ZERO
				goto	bmodb_r1					; ei, päris mitmes oli...
				movf	Char,W
				sublw	modbus_cnf
				btfss	ZERO
				goto	modb_r00				; ei, äkki wr_multi ?
				movlw	.12;7
				movwf	countL
				goto	bRREnd1					; jääb kuuldele...
bmodb_r00:		movf	Char,W
				sublw	modbus_wrmulti
				btfss	ZERO
				goto	bmodb_r1					; ei, siis ei näpi
				movlw	.7
				movwf	countL
				goto	bRREnd1					; jääb kuuldele...


bmodb_r0:
bmodb_r12:
bmodb_r1:
bmodb_r14:
bmodb_r15:
bmodb_r16:
bmodb_r17:		movf	bytecnt,W
				subwf	countL,W				; saadetis käes (countL-s oodatav baitide arv)?
				btfss	ZERO
				goto	bRREnd1					; eip !
				movff	Puhver+.1,WREG
				sublw	modbus_wrmulti			; kas oli käsk 0x10 (kirjuta mitu reg. korraga)?
				btfss	ZERO
				goto	bmodb_r2					; ei, pakett käes, kontrolli summat
				btfsc	cmd10
				goto	bmodb_r2
				movff	Puhver+.6,WREG			; jah, loeme saadetavate baitide arvu ja ootame nende saabumist
				addlw	.2
				addwf	countL,F
				bsf		cmd10
				goto	bRREnd1					; jääb kuuldele...

bmodb_r2:		movlw	0xFF					; pakett käes, kontrollime
				movwf	_RS485chkH				; valmistume kontrollsumma arvutamiseks	
				movwf	_RS485chk
				decf	countL,F
				decf	countL,W
				movwf	bytecnt
				LFSR	.0,Puhver				; kontrollime summat

;				bcf		Dir						; lisa Dir signaal nulli
;				bcf		PIR3,CCP2IF
;				bsf		PIE3,CCP2IE				; ja lubame jälle sellise tekitamise

bmodb_r3:		movf	INDF0,W
				call	bmb_crc16				; kontrollsummeeri
				incf	FSR0L,F
				decfsz	 bytecnt
				goto	bmodb_r3
				movf	_RS485chk,W				; kontroll
				subwf	INDF0,W
				btfss	ZERO
				goto	breset_ser2				; viga
				incf	FSR0L,F
				movf	_RS485chkH,W			; kontroll
				subwf	INDF0,W
				btfss	ZERO
				goto	breset_ser2				; viga, eksiteerib via reset_ser
				bsf		cmd_ok					; aga märgi ära, et pakett oli ok
				bcf		SerialTimerOn			; taimer seisma
				banksel	PIE1
				bcf		PIE1,RC1IE				; enne uut käsku vastuvõtu ei võta kuni senine täidetud
				banksel	.0
				return
;===============================================================================
breset_ser2:		goto	breset_ser
breset_ser1:;		call	reload_side				; sidetaimeri reload
breset_ser:		clrf	bytecnt					; - baitide loendi
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
;				bcf		Dir						; lisa Dir signaal nulli
;				bcf		PIR3,CCP2IF
;				bsf		PIE3,CCP2IE				; ja lubame jälle sellise tekiamise
				return
bRREnd1:			movlw	serialtime				; relae ootetaimer
				movwf	serialtimer
				return
;===============================================================================
; MB_CRC16 -	Will calculate 16bit CRC for MODBUS RTU Packets
;
;		Input: 	Byte for CRC in W
;		Output:	Original byte in W, 
;			CRC16_HI and CRC16_LO new value of CRC16
;===============================================================================
bmb_crc16:		movwf	mb_del1
				movwf	mb_temp2				; store W
				movlw	.8						; 8 bits
				movwf	mb_temp1
				movf	mb_temp2,W				; fetch W
bCrc_Get_Bit:	rrcf		mb_temp2,F				; bit in C
				movf	mb_temp2,W				; value to W
				bnc	bcrc1;skpnc
				goto	bCrc_In_1
bcrc1:			btfss	_RS485chk,.0			; lowest bit set ?
				goto	bCrc_Cont				; goto count with C=0
				bsf		CARRY
				goto	bCrc_Cont				; goto count with C=1
bCrc_In_1:		btfsc	_RS485chk,.0			; lowest bit zero ?
				bcf		CARRY					; if no, C=0 = complement
bCrc_Cont:		bc		bcrc2;skpc
				goto	bCrc_Shift				; if C=0 only shift
bcrc2:			btfsc	_RS485chkH,.6			; complement 15th bit of CRC
				goto	bCrc1
				bsf		_RS485chkH,.6			; if clear, set
				goto	bCrc2
bCrc1:			bcf		_RS485chkH,.6			; if set, clear
bCrc2:			btfsc	_RS485chk,.1			; complement 2nd bit of CRC
				goto	bCrc3
				bsf		_RS485chk,.1
				goto	bCrc_Shift
bCrc3:			bcf		_RS485chk,.1
bCrc_Shift:		rrcf		_RS485chkH,F			; 16bit rotate
				rrcf		_RS485chk,F
				movf	mb_temp2,W
				decfsz	mb_temp1,F
				goto	bCrc_Get_Bit
				movf	mb_del1,W				; fetch the original byte
				return
;===============================================================================
; ***************************** EEPROM *****************************************
;===============================================================================
; **** seadme ID ****
 code_pack 0xF00000								; EEPROM'i sisu 
e_ADR:						db 0x01				; R274 modbussi aadress
e_IDH:						db 0x00				; R258 vidina unikaalne ID, bait 1
e_IDL:						db 0x00				; R259 vidina unikaalne ID, bait 2
e_PUa:						db 0x00				; pullup mask ehk sisuliselt väljundite seis stardil sest PUsid juhitakse väljunditega
e_PUd:						db 0x00				; R272H analoog,R272L digi: pullup mask ehk sisuliselt väljundite seis stardil sest PUsid juhitakse väljunditega
e_ser:						db 0x02				; R273 seriali ja sisendite lugemise parameetrid: vaata allpool
e_IOD:						db 0x00				; R275L ANA-pordi (UIO) suund, 1= väljund, bitthaaval)
e_anadigi:					db 0xFF				; R275H analoogpordi seisund stardil - analoog või digi. 1= digi
e_devtype					db 0xF1				; R256
e_firmwarenr				db 0x02,0x33		; R257H ja L: F/w HIGH ja LOW
e_reset1:					db 0x00,0x00,0x00,0x00		; R276,R277 180 sekundit viidet, 5 sekundit pulsi kestust
e_reset2:					db 0x00,0x00,0x00,0x00		; R278,R279 - sama
e_xor:						db 0x00				; R271H sellega XORitakse ANA-pordi sisendit (et teha juhitav aktiivne HI/LO)
e_xorD:						db 0x00				; R271L sellega XORitakse DIGI-pordi sisendit (et teha juhitav aktiivne HI/LO)
e_ADC:						db 0x30				; R270L ADC Vref-i konf. Vaid bitid 3,4,5 mõjuvad (Selects the special trigger from the ECCP1,Vref=Avdd,A/D VREF-=0,Analog Negative Channel=GND)
e_pwm:						db 0x00				; R270H bitt 0 =1 -> pwm lubatud. Enam seda ei arvesta !
e_pwmperiod:				db 0x00,0x00		; R150 ehk PWMi periood
e_pwmconf:					db 0x00,0x00		; R151 ehk PWMi konf - kui bitt=1 siis see kanal on PWM
e_reg699:					db 0x00,0x00		; kui 1, loetakse 1-juhtme aadressid EEPROMist
e_ds1820_1:					db 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
e_ds1820_2:					db 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
e_ds1820_3:					db 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
e_ds1820_4:					db 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
e_ds1820_5:					db 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
e_ds1820_6:					db 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
e_ds1820_7:					db 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
e_ds1820_8:					db 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
e_ds1820_9:					db 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
e_ds1820count:				db 0x00
e_ds2438_1:					db 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
e_ds2438_2:					db 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
e_ds2438_3:					db 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
e_ds2438_4:					db 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
e_ds2438_5:					db 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
e_ds2438_6:					db 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
e_ds2438_7:					db 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
e_ds2438_8:					db 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
e_ds2438_9:					db 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
e_ds2438count:				db 0x00
; serialport:
;------------
; b0,1,2 - kiirus, default 19200 (010)
; b3 - debounce kõigile sisenditele, default: ON (=0)
; b4 - sticky bitt kõigile sisenditele, default: OFF
; b5 - wiegand B lubatud (PORTB,4 ja 5)
; b6 - wiegand A lubatud (PORTB,6 ja 7)
; b7 - paarsuse bitt, (0= even parity), default EVEN
	end			
;******************************************************************************