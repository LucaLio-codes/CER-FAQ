# CER Selbsttestfragen
Hier sind alle Fragen die am Ender jeder Vorlesung stehen gelistet und von mir Beantwortet. <br>
Alle Antworten sind nach besten Wissen und Gewissen, steinigt mich nicht wenn was Falsch ist <br>
Fragen von SoSe 21 <br>
Gruß geht raus an Jan, der die Fragen selbst formuliert aber nicht gescheit in den Foliensätzen beantwortet 

## Vorlesung 0 Grundlagen


### Was ist eine Simulation?
<details>
    <summary>Antwort</summary>
    Eine Simulation ist der Prozess der Modellierung eines echten System und die ausführung von virtuellen experimenten auf diesem. Zweck der Simulation ist entweder das Nachvolziehen des echten Systems oder das Entwickeln von Strategien für die Anwendung des echten Systems
</details>

### Warum und wofür braucht man Simulation?
<details>
    <summary>Antwort</summary>
    Das echte System ist ..
    <li>zu groß oder klein</li>
    <li>zu schnell oder zu langsam</li>
    <li>noch nicht geabaut</li>
    <li>zu gefährlich</li>
    <li>nicht experimentierbar</li>
    <li>zu teuer</li>
    <li>zu stark gestört</li>
</details>

### Was ist ein Modell?
<details>
    <summary>Antwort</summary>
    Im grunde Alles<br>
    Modellierung ist die Beschreibung der Realität. Dabei verbinden Modelle beobachtbare oder messbare Variabeln so das verständliche Muster entstehen
</details>

### Was haben Modelle mit Wissenschaft zu tun?
<details>
    <summary>Antwort</summary>
    Modelle Ermöglichen testbare Vorhersagen und somit die "Scientific Method"
</details>

### Mit welchen Schritten erstellt man Modelle für ein System?
<details>
    <summary>Antwort</summary>
    <li>Analytische Modellierung: Modellgleichung basierend auf Expertenwissen aufstellen</li>
    <li>Systemidentification: Bestimmen der Parameter durch Experimente</li>
</details>

### Warum braucht auch die Künstliche Intelligenz Simulationen?
<details>
    <summary>Antwort</summary>
    Künstliche Inteligenzen müssen Trainiert werden, dies passiert meist in einem Simulierten umfeld um die Effizienz zu erhöhen
</details>

### Wofür können Modelle noch genutzt werden?
<details>
    <summary>Antwort</summary>
    Blöde frage eigentlich:
    <li>Kunst</li>
    <li>Sofwareentwicklung</li>
    <li>Produktentwicklung</li>
    <li>Verhaltensprognose/-analyse</li>
    <li>Buchstäblich alles</li>
</details>

### Kannst Du das SIR Modell in Python programmieren?
<details>
    <summary>Antwort</summary>

```python
    import numpy as np
    import matplotlib.pyplot as plt
    def sir(u):
        s, i, r = u
        gamma = 0.17
        betta = 0.23
        du = [
            - (betta * s * i),
            betta * s * i - gamma * i,
            gamma * i
        ]
        du = np.array(du)
        return du

    u0 = [1.0, 0.001, 0.0]
    n = 200
    data = []
    u_old = u0
    data.append(u_old)

    for i in range(200):
        du = sir(u_old)
        tmp = u_old + du
        data.append(tmp)
        u_old = tmp
    plt.plot(data)
    plt.legend(["S","I","R"])
    plt.show()
```
</details>

### Wie könnte ein SIEARS (A=Asymptomatic, zweites S=Rückfälle) aussehen?
<details>
    <summary>Antwort</summary>
    Grundidee: neue konstante für rückfall (rho) und asympt (alpha)<br>
    mehr weiß ich auch nicht

</details>

## Vorlesung 1 Modellierung/ Vorwärtskinematik

### Was sind Eigenvektoren und wozu braucht man die?
<details>
    <summary>Antwort</summary>
    "lambda, v sin Eigenwert und der Dazugegörige Eigenvektor von A gdw Av = lambda v"
    "Eigenvektoren sind nützlich um Matrizen zu diagonalisieren" 

</details>

### Wie geht der Modellierungsloop?
<details>
    <summary>Antwort</summary>

```
    ˍˍˍˍˍˍˍˍˍˍˍˍˍˍˍˍˍˍˍˍˍˍˍˍˍˍ                        ˍˍˍˍˍˍˍˍˍˍˍˍˍˍˍˍˍˍˍˍˍˍˍˍ
    |Problem aus der Realität| -----Modellbildung---> |Mathematisches Problem|
    ¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯                        ¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯
    ^                                                                   |
    |                                                                   |
    Überprüfung                                     Anwenden der Egrebnisse
    |                                               / Methoden
    |                                                                   |
    |                                                                   v
    ˍˍˍˍˍˍˍˍˍˍˍˍˍˍˍˍˍˍˍˍˍˍˍˍˍˍˍˍˍ                       ˍˍˍˍˍˍˍˍˍˍˍˍˍˍˍˍˍˍˍˍˍ
    |Sachverhalt in der Realität|<--Interpretation------|Lösung des Problems|
    ¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯                       ¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯
```

</details>

### Wie unterscheiden sich statische und dynamische Modelle?
<details>
    <summary>Antwort</summary>
    Statische Modelle stellen Verschiedene Größen im Modell in Beziehung oder beschreibt eine abhängige Größe mit Hilfe einer anderen, das Model hängt also nur von diesen Größen ab<br>
    Dynamische Modelle hängen von dem jetzigen zustand, dem vorhergegangenen und diesen Werten ab (man betrachtet die ableitungen)
</details>

### Was bedeuten zeitdiskret, ortskontinuierlich, stochastisch?
<details>
    <summary>Antwort</summary>
    <li>zeitdiskret:  Das Modell beschreibt das System für bestimmte abzählbare Zeitpunkte.</li>
    <li>ortskontinuierlich: Das Modell beschreibt das System für kontinuierliche (unendlich viele nicht abzählbare) Orte.</li>
    <li>stochastisch: Zustand wird als verauschte Punktmasse betrachtet</li>
</details>

### Welche Fragestellungen behandelt die Kinematik?
<details>
    <summary>Antwort</summary>
    <li>vorwärts: mit gegebenen Gelenkparameter q: wo steht unser Endeffektor</li>
    <li>rückwärts: mit welchen Gelenkparameter q erreicht unser Endeffektor die position x</li>
</details>

### Wie berechnet man Vorwärtskinematik?
<details>
    <summary>Antwort</summary>
    Für jedes Gelenk stellt man ein eigenes Koordinatensystem S<sub>i</sub> auf. Von den Gelenkvariablen q abhängige Transformationsmatrizen <sup> i-1 </sup>T<sub>i</sub> geben die Transformation von einem Koordinatensystem S <sub> i </sub> in ein Koordinatensystem S <sub>i-1</sub> an. Durch ihre Verkettung (durch Multiplikation) erhält man die Transformationsmatrix <sup>0</sup>T<sub>n</sub>, die aus der Position eines Punktes <sup>n</sup>p bezüglich des Endeffektor-Koordinatensystems dessen Position <sup>0</sup>p bezüglich der Basis (des globalen Koordinatensystems) berechnet.
</details>

## Vorlesung 2 Inverse Kinematik

### Warum ist inverse Kinematik schwerer als Vorwärtskinematik?
<details>
    <summary>Antwort</summary>
    <li>Vorwärts: eine Explizite Lösung</li>
    <li>invers: unendlich viele Lösungen möglich (oder auch keine)</li>
</details>

### Wieviel Freitheitsgrade hat ein Auto?
<details>
    <summary>Antwort</summary>
    Drei Freiheitsgrade (auf Ebene Projiziert):
    <li>x</li>
    <li>y</li>
    <li>Fahrzeugorientierung</li>
</details>

### Wenn eine Schlange (mit 200-435 Wirbeln) sich aufrichtet, wiev iele inverse Kinematik-Lösungen hat sie?
<details>
    <summary>Antwort</summary>
    unendlich viele 
</details>


### Was macht das Fixpunktverfahren? Wann konvergiert es?
<details>
    <summary>Antwort</summary>
    <li>Das itterative anwenden einer Funktion auf sich selbst mit startwert x_0 zum Ermitteln einer Nullstelle</li>
    <li>wenn d(g(x),g(y) <= kd(x, y) für alle x,y in R^n mit k in [0,1)</li>
    <li>also wenn alle eigenwerte der Jacobimatrix kleiner 1 (im Einheitskreis)</li>
</details>

### Was ist das Relaxationsverfahren? Wann ist es besser/schlechter als das Fixpunktverfahren?
<details>
    <summary>Antwort</summary>
    Beim Relaxationsverfahren wird die Funktion f(x) mit einer Relaxationsmatrix A multipliziert, damit das Fixpunktverfahren konvergiert<br>
    <li>besser: bestimmen von lokaler Konvergenz bei start x in Nähe von Lösung</li>
    <li>schlechter: bestimmen der globalen Konvergenz</li> 
</details>


### Was macht das Newtonverfahren?
<details>
    <summary>Antwort</summary>
    Das Newtonverfahren ist eine Fixpunktiteration mit adaptiver Relaxationsmatrix
</details>

### Welche Verfahren sind 1. Ordnung, welche zweiter Ordnung?
<details>
    <summary>Antwort</summary>
    FPI und Relaxationsmatrix: 1. Ordnung <br>
    Newton: 2. Ordnung
</details>

### Wie kann ich das Newtonverfahren nutzen um ein Optimierungsproblem max_x s_{Note}(x) zu lösen? Z.B. für s_{Note}(x) = exp(−x) (1 − exp(−x))^{−2}? Tip: Nullstelle von s'_{Note}(x) = 0
<details>
    <summary>Antwort</summary>
    Mit f(x) = s’(x) = 0 erhält man ein Nullstellenproblem. Die Funktion f(x) muss noch einmal abgeleitet werden, um das Newtonverfahren x<sub>i+1</sub> = x<sub>i</sub> – f(x<sub>i</sub>)/f’(x<sub>i</sub>) anwenden zu können.
    Allerdings sieht man schon vor Durchführung des Verfahrens, dass ein Extremum bei x=0 liegt, wo die Funktion gegen Unendlich geht. Das Plotten des Graphen macht deutlich, dass es keine weiteren Extrema gibt.
</details>

## Vorlesung 3 Dynamische Modelle

### Was ein typisches Modell für die Federkraft?
<details>
    <summary>Antwort</summary>
    F<sub>Feder</sub> = k<sub>p</sub> (x<sub>ref</sub>-x) <br>
    mit k<sub>p</sub> als Federkonstante
</details>

### Was ein typisches Modell für Reibung?
<details>
    <summary>Antwort</summary>
    F<sub>Dämpfer</sub> = -k<sub>d</sub>x' <br>
    mit k<sub>d</sub> als Dämperfkonstante
</details>

### Wie wirken sich Kräfte auf die Position eines Körpers aus?
<details>
    <summary>Antwort</summary>
    Kräfte ändern die geschwindigkeit eines Körpers und somit auch die Position<br>
    F = mx''
</details>

### Wie wirken sich Kräfte auf die Orientierung eines Körpers aus?
<details>
    <summary>Antwort</summary>
    Kräfte können drehmomente erzeugen, welche wiederum den Körper drehen <br>
    Summe der Dremomente F<sub>i</sub> r<sub>i</sub> = Trägheitsmoment mal Winkelbeschleunigung
</details>

### Müssen wir bei Punktmassen die Orientierung modellieren?
<details>
    <summary>Antwort</summary>
    Nein
</details>

### Was ist die Dynamikgleichung eines Feder-Masse-Dämpfer Systems?
<details>
    <summary>Antwort</summary>
    <img src="bilder/Feder-Masse-Daempfer.PNG" alt="Feder-Masse-Daempfer">
</details>

### Was ist die Dynamikgleichung eines Pendels?
<details>
    <summary>Antwort</summary>
    <img src="bilder/pendel.PNG" alt="Pendel">
</details>


### Wie kann ich ein einfaches mechanisches System modellieren?
<details>
    <summary>Antwort</summary>
    <ol>
        <li>Mögliche Vereinfachungen treffen (Körper als Punktmassen darstellen, Seil als Stab annehmen, …)</li>
        <li>Entscheiden, ob man mit Kräften und Positionen oder mit Momenten und Winkeln arbeitet</li>
        <li>Alle wirkenden Kräfte/ Momente ermitteln (Auch Dämpfung!)</li>
        <li>Masse/ Trägheitsmoment ermitteln</li>
        <li>In Newtons 2. Gesetz einsetzen</li>
    </ol>
</details>

## Vorlesung 4

### Bestimme die Gleichgewichtslösungen des SIR-Modells? Was kann man den Politikern raten?
<details>
    <summary>Antwort</summary>
    Für eine Gleichgewichtslösung muss die Änderung von s, i und r null sein. Aus dr/dt = γ i = 0 folgt bei gegebenem γ, dass i = 0 sein muss. Dadurch werden auch die beiden anderen Gleichungen 0. Also sind alle Zustände mit i=0 Gleichgewichtslösungen.

    Die Pandemie kann also nicht in einem Gleichgewicht sein, solange es noch infizierte Personen gibt.
</details>

### Kann eine ODE dritter Ordnung auf eine ODE erster Ordnung reduziert werden?
<details>
    <summary>Antwort</summary>
    ja, einführen von einer neuen Zustandsvariable wobei z = (x, x', x'') und somit z' = z(x', x'', x''')

</details>

### Was bedeutet autonom? Was bedeutet zeitinvariant?
<details>
    <summary>Antwort</summary>
    autonom: die Zeit t komm nurnoch parametrisch vor <br>
    zeitinvariant: die zeit kommt nicht vor

</details>

### Was ist ein lineares System?
<details>
    <summary>Antwort</summary>
    Bei linearen ODEs ist die Funktion f(x(t), u(t)) linear

</details>

### Wie kann man die Dynamik eines Roboters mit steifen „Links“ aufschreiben?
<details>
    <summary>Antwort</summary>
    als lineare ODE 1. Ordnung
</details>

### Wozu braucht man statische Modelle beim Brückenbau und wozu dynamische?
<details>
    <summary>Antwort</summary>
    statisch: wie lang wird meine Brücke, wie viel gewicht hällt sie <br>
    dynamisch: wie verhält sich meine Brücke im Wind
</details>

### Wenn ich eine Brücke beschreiben möchte, was für eine Differentialgleichung brauche ich?
<details>
    <summary>Antwort</summary>
    Partielle DGL 2.Ordnung (?)
</details>

### Was ist ein stationärer Zustand und wie hängt er mit der Gleichgewichtslösung zusammen?
<details>
    <summary>Antwort</summary>
    Ein zustand in dem die beschleunigung und geschwindigkeit = 0 ist. <br>
    eine gleichgewichtslösung ist ein Kandidat für einen stationären Zustand
</details>

## Vorlesung 5 Stabilität
### Was bedeutet Linearisierung? Wann kann ich sie anwenden?
<details>
    <summary>Antwort</summary>
    Approximieren eines Systems x'(t) = f(x(t), u(t)) in der nähe eines Arbeitspunktes durch eine lineares x' = Ax +Bu<br>
    Kann angewendet werden bei Systemen mit Gleichgewichtslösungen <br>
    nicht anwendbar wenn der AP in einer Unstetigkeit in der 1. oder 2. Ableitung hat (sprung oder knick)
</details>

### Wie kann ich ein lineares System analysieren? Was bedeuten Eigenwerte und -vektoren in diesem Zusammenhang?
<details>
    <summary>Antwort</summary>
    Mit Eigenwerten und Eigenvektoren sind in diesem Fall die Eigenwerte und -vektoren der Matrix A in x’= Ax + Bu gemeint.
    Die Eigenwerte zeigen, ob das System stabil ist und ob es oszilliert. Die Eigenvektoren geben die Pfeilrichtung im Phaseplane an.
</details>

### Was sagt der Imaginärteil der Eigenwerte über Oszillationen aus?
<details>
    <summary>Antwort</summary>
    Gibt es mindestens einen komplexen Eigenwert, schwingt das System. Sind alle Realteile 0, aber nicht alle Imaginärteile, so findet theoretisch unendliche Oszillation statt. Das ist für linearisierte Systeme (die eigentlich nichtlinear sind) aber nicht garantiert.
</details>

### Was sagt der Realteil der Eigenwerte über Stabilität aus?
<details>
    <summary>Antwort</summary>
    nur wenn alle Realteile der Eigenwerte negativ sind ist das System stabil
</details>

### Was passiert bei einem linearen System bei Re(λ) = 0 und Im(λ) != 0?
<details>
    <summary>Antwort</summary>
    dauerhafte oszilation
</details>

### Was passiert bei einem nichtlinearen System bei Re(λ) = 0 und Im(λ) != 0?
<details>
    <summary>Antwort</summary>
    oszilation nicht grarantiert
</details>

### Wie sieht das Zeitverhalten eines van der Pol-Oszillators aus?
<details>
    <summary>Antwort</summary>
    zeitinvariant
</details>

## Vorlesung 6 PD Regler

### Kann man per „Software“ eine künstliche Feder oder Dämpferhinzufügen?
<details>
    <summary>Antwort</summary>
        Komt auf den Kontext an. In einer Simulation, klar, der Ganze ablauf ist künstlich.
        Am echten System braucht man eine möglichkeit kräfte wirken zu lassen. Die Programmierte feder könnte z.B anhand eines Bildes des Systems einen Motor steuern und somit
        eine Feder simulieren
</details>

### Was ist der Unterschied zwischen Steuerung und Regelung?
<details>
    <summary>Antwort</summary>
    Steuerung ist zeitabhänig vorbestimmt (open loop), Regelung ist nur abhängig vom vorhergehenden Zustand (closed loop)
</details>

### Was ist der Unterschied zwischen Trajektorienregelung und „Set Point“ Regelung?
<details>
    <summary>Antwort</summary>
    der Unterschied zwischen "Set-Point" und Trajektorienregelung ist ganz einfach. Bei der Trajektorienregleung ist der gewuenschte Systemzustand zeitabhaengig, waehrend bei er bei "Set-Point" Regelung konstant ist.<br>
    Beispiel Trajektorienregelung: Ein humanoider Roboter soll mit einer vorgegebenen Geschwindigkeit laufen.<br>
    Beispiel Set-Point: Der gleiche humanoide Roboter soll auf einem Bein stehen bleiben und moeglichst wenig wackeln.

</details>

### Was ist ein PD-Regler? Was würde einen PID-Regler ausmachen?
<details>
    <summary>Antwort</summary>
    P = Proportional (die Feder; Positionsanteil)  <br>
    D = Derivative (der Dämpfer; Geschwindigkeitsanteil) <br>
    I = Integral (Akkumulierter Fehler)
</details>

### Was ist ein Zustandsregler?
<details>
    <summary>Antwort</summary>
    ein PD reger in Matrixform also u = Kx
</details>

### Was bedeuten open-loop und closed loop?
<details>
    <summary>Antwort</summary>
    open: steuerung; steuergrößen abhängig von Zeit <br>
    closed: regelung; Abhängig von zustand x(t) (Feedback)
</details>

### Was bedeutet feedforward und was feedback?
<details>
    <summary>Antwort</summary>
    feedback: Closed-Loop regelung <br>
    feedforward: open-loop Steuerung
</details>

### Wie kann man den geschlossenen Regelkreis als Differentialgleichung beschreiben?
<details>
    <summary>Antwort</summary>
    x' = f(x, u(x)) <br>
    mit u(x) = K<sub>D</sub>(x'<sub>des</sub>(t) - x') + K<sub>P</sub>(x<sub>des</sub>(t) - x) + K<sub>I</sub> [Integral] (x<sub>des</sub>(t) - x dt)
</details>

### Was ist (über-/unter-)kritische Dämpfung?
<details>
    <summary>Antwort</summary>
    Ein System ist ...
    ... bei einer doppelten, reelen Nullstelle kritisch gedämpft => xdes wird schnell ereicht ohne überschwingen
    ... bei einer einfachen, reelen Nullstelle überkritisch gedämpft => langsamer aber ohne überschwingen
    ... bei einer doppelten, komplexen Nullstelle unterkritisch gedämpf => schnell aber überschwingen
</details>

## Vorlesung 7 Numerische Integration

### Warum ist numerische Integration notwendig?
<details>
    <summary>Antwort</summary>
    zum Lösen von nichtlinearen DGLs
</details>

### Wie liest man das zeitliche Verhalten aus einem Richtungsfeld?
<details>
    <summary>Antwort</summary>
    wan folgt den Pfeilen vom startpunkt in positiver x-Richtung
</details>

### Was unterscheidet ein explizite und implizite Verfahren?
<details>
    <summary>Antwort</summary>
    explizites Verfahren: x_k+1 nur abhängig von x_k <br>
    implizites Verfahren: x_k+1 von sich selbst abhängig

</details>

### Was sind Einschrittverfahren und deren Konsistenzbedingung?
<details>
    <summary>Antwort</summary>
    Einschrittverfahren haben die Form x_k+1 = x_k + h * Verfahrensfunktion <br>
    Konsistenzbedingung : für h -> 0 ist Verfahrensfunktion -> f(x_k)
</details>

### Warum sind Verfahren 1. Ordnung prinzipiell problematisch?
<details>
    <summary>Antwort</summary>
    weil sich approximationsfehler propagieren, sprich in jedem Schritt größer werden
</details>

### Was sind Euler-Verfahren?
<details>
    <summary>Antwort</summary>
    Euler Verfahren sind einschrittverfahren erster Ordnung:
    <li>Explizit: x_k+1 = x_k + h f(x_k)</li>
    <li>Implizit: x_k+1 = x_k + h f(x_k+1)</li>
</details>

### Was ist das Heun-Verfahren? Wann sollte ich es einsetzen?
<details>
    <summary>Antwort</summary>
    Verfahren 2. Ordnung.
    <img src="bilder/Heun.PNG" alt="Heun">
    sollte genutzt werden wenn man bei niedriger Anzahl an stützpunkten bessere Ergebnisse als bei Euler braucht
</details>

### Was ist das Runge-Kutta-Verfahren? Wann verwendet man es?
<details>
    <summary>Antwort</summary>
    Verfahren der 4. Ordnung. <br>
    <img src="bilder/RK4.PNG" alt="RK4">
    höherer Zeitaufwand für mehr Präzision
</details>

### Wodurch entstehen Approximationsfehler?
<details>
    <summary>Antwort</summary>
    <li>durch die Integralaproximation</li>
    <li>durch runden</li>
</details>

### Was ist Schrittweitensteuerung?
<details>
    <summary>Antwort</summary>
    Die Schrittweite wird so angepasst, dass das Ergebnis möglichst genau wird und sowohl Rundungsfehler als auch Integralapproximationsfehler klein werden. Eventuell verwendet man variable Schrittweiten.
</details>


## Vorlesung 8 Numerische Aspekte der Simulation von ODEs

### Wann heisst eine ODE steif?
<details>
    <summary>Antwort</summary>
    Wenn Tmax deutlich größer Tmin; also die schnellste Zeitcharacteristik deutlich schneller der langsamsten <br>
    Faustregel: wenn Tmax/Tmin >= 10² dann ist ODE steif 
</details>

### Warum ist das problematisch?
<details>
    <summary>Antwort</summary>
    wirkt sich negativ auf die numerische Lösbarkeit aus, niedrigere Schrittweite wird benötigt. (Mehr berechnungen)
</details>

### Gib mehrere praktische Beispiele für steife ODEs!
<details>
    <summary>Antwort</summary>
    Klimamodelle incl Wetter. Das Wetter aendert sich sehr schnell, waehrend die Effekte von Treibhausgasen usw haeufig Zeitcharakteristika in der Groessenordnung von Jahrzehnten haben.
</details>

### Warum sind unstete ODEs wichtig?
<details>
    <summary>Antwort</summary>
    Sie werden benötigt um z.B einen Aufprall, eine Gangschaltung, Ventile zu Modellieren
</details>

### Was für Probleme haben sie?
<details>
    <summary>Antwort</summary>
    An der Unstetigkeit existiert keine Ableitung => Numerische Lösungsverfahren verlangen aber nach stetiger Differenzierbarkeit
</details>

### Wie werden Zahlen dargestellt?
<details>
    <summary>Antwort</summary>
    <li>für Menschen: Je nach Anwendung verschieden genau, fast immer Base 10</li>
    <li>für Rechner: Je nach Anwendung genau base 2, hier ist wahrscheinlich der IEEE 754 Float gemeint, der nach dem schema [Kommazahl][Potenz] Aufgebaut ist</li>
</details>

### Warum ist die Ariane 5 abgestürzt? Was hat das mit Rundung zu tun?
<details>
    <summary>Antwort</summary>
    64 Bit float to 16 Bit int Conversion
</details>

### Wie setzen sich Rundungsfehler fort? Was hat das mit Konditionierung zu tun?
<details>
    <summary>Antwort</summary>
    z.B bei Expliziten Verfahren wo der Fehler in die nächste iteration mitgenommen wird und sich addiert. <br>
    Gut konditionierte Funktionen sind nicht so anfällig, da kleine Abweichung in der Eingabe auch kleine Änderungen in der Ausgabe nach sich führen.
</details>

### Wie hängen Mess- und Rundungsfehler zusammen?
<details>
    <summary>Antwort</summary>
    beides sind Fehler, beide können gleichzeitig auftreten
</details>

### Welche Fehlerart ist relevanter in der Praxis?
<details>
    <summary>Antwort</summary>
    Gerundet wird einmal am Ende, gemessen wird jede relevante Variable, daher tät ich sagen der Messfehler<br>
    "In der Robotik sicher die Messfehler. Das Rauschen auf realen Sensordaten 
    ist viel groesser als die Praezision von 64-bit floating point Zahlen."
</details>

## Vorlesung 9
### Was ist die „Methode der kleinsten Quadrate“ (Least Squares)?
<details>
    <summary>Antwort</summary>
    Die quadratische Abweichung zwischen Funktionswert und gemessenem Wert soll minimiert werden. <br>
    "Fast richtig. Wichtig ist noch, dass es in der Regel um mehrere Datenpunkte geht und die Summe aller Quadrate minimiert werden soll und nicht das geometrische Mittel oder aehnliches."
</details>

### Welche Kostenfunktion ist hier gemeint?
<details>
    <summary>Antwort</summary>
    L(θ) = 1/N Σ<sup>N</sup><sub>i=1</sub> (y<sub>i</sub> - f<sub>θ</sub> (x<sub>i</sub>))²
</details>

### Welche Annahmen liegen „Least Squares“ zu Grunde?
<details>
    <summary>Antwort</summary>
    kurz: exaktes Messen <br>
    lang: 
    <li>Unabhängige Datenpunkte</li>
    <li>Datenpunkte stammen alle aus der gleichen Gaussverteilung</li>
    <li>perfekte Messung der Eingangsdaten, ABER</li>
    <li>Rauschen (Gauss verteilt) auf den Ausgangsdaten ist erlabut!</li>
    <li>Eingänge nicht linear korelliert</li>
    <li>Rauschen des Ausgangs ist nicht linear korelliert</li>
</details>

### Welche Annahmen werden bei realen Systemen oft verletzt?
<details>
    <summary>Antwort</summary>
    <li>Kein Rauschen am Eingang</li>
    <li>kein Rauschen am Ausgang</li>
</details>

### Was sollte man tun, wenn die Eingangsdaten verrauscht sind?
<details>
    <summary>Antwort</summary>
    Filtern, aber vorsichtig; Anderes Schätzverfahren
</details>

### Ist Least Squares robust gegenüber verrauschten Ausgangsdaten?
<details>
    <summary>Antwort</summary>
    Es darf nur nicht linear Korelliert sein
</details>

### Warum sollte man mehrmals mit unterschiedlichen Daten schätzen?
<details>
    <summary>Antwort</summary>
    Um zu verhindern dass die gewählten Parameter nur auf den gewählten Daten passt sondern tatsächlich das System beschreibt (kinda overfitting) <br>
    Um mit Mess- und Schätzfehlern umzugehen, da diese im Zweifelsfall abweichende Ergebnisse hervorrufen.
</details>

## Vorlesung 10: Black-Box-Modelle

### Warum ist nichtlinear least squares schwerer als linearer least squares?
<details>
    <summary>Antwort</summary>
    für die nichtlineare least squares gibt es keine Analytische Lösung, folglich muss eine Lösung mit passenden Tools erarbeitet werden
</details>

### Warum ist White-Box System Identification immer unvollständig?
<details>
    <summary>Antwort</summary>
    Da man nicht alle Parameter kennt, oder selbst wenn nicht alle leicht modellierbar sind.
</details>

### Warum sind „unvollständige“ Modelle gefährlich?
<details>
    <summary>Antwort</summary>
    können systematisch die Parameterschätzung verfälschen (Think Jan und sein Roboter den er direkt geschrottet hat)
</details>

### Was ist an realen Systemen schwer zu modellieren?
<details>
    <summary>Antwort</summary>
    Reibung, Softbody Kollisions, Effektde des Gewählten Antriebs (Gerbox/ Kabel etc.)
</details>

### Was für Black-Box Basis Funktionen gibt es?
<details>
    <summary>Antwort</summary>
    unzählig viele, in der Vorlesung genannt:
    <li> Monome </li>
    <li> Cosinusterme</li>
</details>

### Was sind die Probleme mit Basis-Funktionen? Was passiert bei zu vielen oder zu wenigen?
<details>
    <summary>Antwort</summary>
    Man braucht für jedes Problem die richtigen Basisfunktionen. Das Problem muss sich als Linearkombination der Basisfunktionen darstellen lassen. <br>
    Man hat keine Möglichkeit, die Richtigkeit des Ergebnisses mit „Menschenverstand“ zu überprüfen. Eventuell bekommt man unvorhergesehene Ergebnisse.<br>
    Zu wenige: Die Ergebnisse sind Qualitativ schlecht <br>
    zu viele: overfitting (?)
</details>

### Wie unterscheiden sich neuronale Netze von Basis Funktion-Ansätzen?
<details>
    <summary>Antwort</summary>
    Neuronale Netze lernen in Hidden Layers selbstständig ihre Features. Daher kann man aber auch schlechter interpretieren, was sie gelernt haben.
    Bei neuronalen Netzen werden mehrere Systeme hintereinander geschaltet, sodass die Ausgabe von einem System die Eingabe des nächsten ist.
</details>

### Was für Funktionen können sie repräsentieren?
<details>
    <summary>Antwort</summary>
    Neuronale Netze können alle stetigen Funktionen auf einem kompakten Intervall repräsentieren.
</details>

### Wie viele Basis Funktionen kann ein neuronales Netz repräsentieren?
<details>
    <summary>Antwort</summary>
    alle?
</details>

## Vorlesung 11: Verifikation und Validierung

### Was für Fehlerquellen gibt es bei der Parameterschätzung?
<details>
    <summary>Antwort</summary>
    <li>Modellierungsfehler
        <ul>
            <li>Ungenauigkeit in Modellparametern</li>
            <li>vereinfachte Modellannahmen </li>
        </ul>
    </li>
    <li>Fehlerakkumulation durch wiederholten Einsatz des Modells</li>
    <li>Approximationsfehler des iterativen Berechnungsverfahrens</li>
    <li>Rundungsfehler</li>
    <li>Programmier-, Implementierungsfehler</li>
</details>

### Wie kann man mit Modellfehlern umgehen?
<details>
    <summary>Antwort</summary>
    Validieren oder Verifizieren
</details>

### Was bedeuten Verifikation und Validierung?
<details>
    <summary>Antwort</summary>
    Verifikation: Formaler Nachweis dass ein Modell eine vorgegebene Spezifikation entspricht <br>
    Validierung: Plausibilitätsprüfung des selbigen durch testen und Beispiele
</details>

### Wie unterscheidet sich Verifikation von Programmen zu Verifikation von Modellen? Gibt es ein „wahres“ Modell?
<details>
    <summary>Antwort</summary>
    Viele Programme lassen sich Verifizieren, siehe FMISE. <br>
    Ein Modell ist immer nur so genau wie man es aufstellt, das "wahre" Modell existiert nicht
</details>

### Wie unterscheidet sich Validierung von Programmen zu Validierung von Modellen?
<details>
    <summary>Antwort</summary>
    validierung ist bei Modellen im gegensatz zu Software nicht nur Testen, man kann auch Hypotesen aufstellen und Intuitionen nachvollziehen
</details>

### Was sind Test- und Trainingssets? Wie generiert man diese Datensätze?
<details>
    <summary>Antwort</summary>
    Eine Ansammlung von Eingabe und Ausgabe Werten, die getrennt voneinander einmal zum Trainieren des Modells (parameterschätzung) und einmal zur Validierung dieser genutzt werden<br>
    Es gibt viele Möglichkeiten, diese Datensätze zu generieren, beispielsweise teilt man die gemessenen Daten zufällig auf oder aber nach Trajektorien. Auf jeden Fall dürfen sich die Datensätze nicht überschneiden, da die Tests sonst nicht aussagekräftig sind.
</details>

### Was passiert wenn der Testfehler hoch und der Trainingsfehler niedrig ist?
<details>
    <summary>Antwort</summary>
    Overfitting: Das Modell kan gut mit der Gegebenen Trajektorie umgehen, diese scheint aber nicht repräsentativ für das System zu sein
</details>

### Wann sollte man White-Box Modelle nehmen? Wann nicht?
<details>
    <summary>Antwort</summary>
    Soweit wie es geht. Schwer Modellierbare Effekte sollten dann als Black-Box Modell modelliert werden = Grey-Box
</details>

