# Przesouwanie figur
## Sterowanie robotem
Użycie moveit task constructor
Moveit odpowiada za planowanie trajektorii, dzięki task constructor można połączyć trajektorie np przesunięcie pionowo w góre z pójściem do pozycji. Problem w integracji własnego sterowania chwytakiem z tym systemem. Prędkość wykonywania może być wolna przy duży ograniczeniach i liczbie rozwiązań. Algorytmy wykorzystywane są iteracyjne więc mogą nie znaleźć optymalnego rozwiązania lub jakiegokolwiej nawet jeśli takie istnieje. Konieczne jest dobre dobranie parametrów tak aby osiągnąć kompromis pomiędzy prędkością a dokładnością.

Próbowano zaimplementować kinematykę odwrotną jednak rezultaty nie były zadowalające więc użyto gotowego rozwiązania do planowania trajektorii.

Pewnie trzeba opisać jak działa to planowanie trajektorii bo inaczej pszypał.
## Sterowanie chwytakiem
Sterowanie po TCP/IP łączymy się z ESP32. Dzięki własnej bibliotece jest możliwość wysyłania komend do płytki sterującej serwami. Możliwość odczytywania pozycji, zadawania pozycji. Do realizacji sterowania z poziomu ROS2 stworzono service który otrzymuje komende otwórz/zamknij i wykonuje akcje. Po sukcesie zwraca komunikat. Sukces imformacja zwortna lub odzczytanie pozycji po jakimś czasie.
## Wysyłanie ruchów
Stockfish jest odpowiedzalny za wymyślanie posunięć, następnie ruch jest rozbijany na składowe które są wykonywane po sobie:
* Normalny ruch (e4e5):
    1. mov_e4
    2. close_grp
    3. mov_e5
    4. open_grp
    5. mov_home
* Bicie (e4d5):
    1. mov_d5
    2. close_grp 
    3. mov_out 
    4. open_grp 
    5. mov_e4 
    6. close_grp
    7. mov_d5
    8. open_grp
    9. mov_home
* Roszada = 2 * normaly ruch
* Promocja - na ten moment nie przewidywana, do rozważenia bo jest to bardzo ważny element gry (może asysta człowieka??) ale będzie problem z rozpoznawaniem figur.
## pomysły
* Dodać planowanie trajektorii w czasie zamykania chwytaka. Wykonanie trajektorii dopiero po.
* Sterownik do chywtaka zgodny z ros2_controll pozwoli na zintegrowanie chwytania z przesuwaniem. Bardzo skomplikowane, mogą być problemy z realizacją tak skomplikowanej trajektorii.
## TODO

##  Detekcja wykonanego ruchu
Schemata wykrywania ruchu

0. Inicjalizacja, znalezienie szachownicy i zapamiętanie rogów do późniejszej transformacji szachownicy
1. Sprawdzenia czy nastąpiła zmiana, jakikolowiek ruch względem obrazu po ruchu robota
2. Sprawdzenie czy kolejne 2-3 klatki są stabline, nie wystąpił żaden ruch
3. Transformacja obrazu tak aby szachownica była na cały ekran
4. Odjęcie obrazów żeby sprawdzić jaki ruch został wykonany
5. Na podstawie poprzedniej pozycji i zmiany wykrycie jaki ruch został wykonany. Kilka przypadków w zależności od typu ruchu
6. Weryfikacja legaloności ruchu
7. Komunikat o sukciesie i jaki ruch został wykonany
8. Ruch robota, następnie przejdź do 1.

Detekcja transformacji szachownicy:

1. Preprocesssing: zmniejszenie obrazu, konwersja do skali szarości, gaussian blur, bilaterlar filter w celu usunięcia szumów i ostrych krawędzi
2. Canny edge detector i hough line transform, aby wykryć linie na szachownicy.
3. Filtracja pobranych linii, wyciągnięcie średniej z tych znajdujących się blisko siebie. Następnie odrzucenie tych które raczej nie są liniami wyznaczającymi pola. Różnica między kolejnymi mniejsza od mediany. 
4. Na podstawie najbardziej wysuniętych linii wyznaczenie obrysu szachownicy
5. Transformacja szachownicy i podział na grid 8x8

### Pomysły
* Ignorować małe zmiany, bo człowiek mógł poprawiać figury. 
* Wykrywać zmiany przed czy po transformacji? Chyba lepiej po bo ktoś mógł coś postawić obok szachownicy.
* 

## MISC
* Gra z robotem działa, dobrze rozpoznaje ruchy białych pionków
* Poprawić rozpoznawanie ruchów, roszada, itp
* Sprawdzić czy ruch jest legalny przed jego wysłaniem
* Chwytak z gąbek

## Sugenstie promotora
* Wykrywanie kontaktu z człowiekiem przez ograniczenie momentów na przegubach
* Dwustopniowe bezpieczeństwo, jeszcze system wizyjny
* Pionek upusczany na szachownice a nie stawiany


### MakeMove service
* move_type = {promotion, capture_promotion, castle, capture, regular, enpassant}
* from_sqr = "e2"
* to_sqr = "e2"
* moved_pice = {p, n, b, r, q, k}
* captured_piece = {p, n, b, r, q, k} opcjonalnie
---
* success
* message

### Gripper cmd service
* open
* close(piece)

## Plan działania 30.11
* Teoria - CNN DONE
* Teoria inne algorytmy? (projekcja, znajdowanie szachownicy) DONE - NIE
* Podrodzdział znajdowanie transformacji szachownicy HALF DONE
* Podrozdział wykrywanie posunięć
* Integracja systemu
* Abstrakt
* Zawartość pracy