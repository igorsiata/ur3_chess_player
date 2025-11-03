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
Dodać funkcje która na podstawie wykonywanego ruchu rozbija go na składowe.

##  Detekcja wykonanego ruchu