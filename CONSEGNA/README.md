# XBOOM-Project
@Autori
- Lorenzo Ciuti
- Simone Cruciani
- Raffaele Tirotta
- Carlo Cordoni
- Francesco Negroni

Contenuto
- _XBoomReport.pdf 
- _SimulatoreMatlab 
- _Letteratura
- _Blender
- _SolidWorks

## Simulatore su MATLAB@
Xboom è un programma sviluppato su matlab capace di simulare la traiettoria di un boomerang seguendo la parametrizzazione. Il simulatore è diviso in:
- File Main
- BlackBox: cartella contente tutte le funzioni implementate
- .stl: file stl di prova

### BlackBox:
1. _Profili, cartella contenente alcuni profili generici
2. _Plot, cartella conentenet alcuni script utili per la visualizzazione della traiettoria simulata:
	- `PlotTopDxSx.m`: visualizza la traiettoria del boomerang sia delle estremità che del baricentro 
	- `plotNy.m`: permette di avere più assi y sulla stessa figura con unità di misura differenti
	- `PlotAeroForce.m`: visualizza le forze ed i momenti aerodinamiche subite dal boomerang nel sistema di riferimento Body o non-inerziale durante il volo
	- `InitialConditionPlot.m`: visualizzazione grafica della posizione del boomerang nel sistema di riferimento non inerziale
	- `FinalPlot.m`: visualizzazione delle variabili di stato della traiettoria (x,y,z,r,p,q e i tre angoli di Eulero)
	- `Energy.m`:  visualizzazione dell'andamento dell'energia meccanica, cinetica e potenziale durante la simulazione, calcolandoli seguendo la formula 48 del report al capitolo 4.3
	- `ChiAvan.m`: Visualizzazione del parametro Chi spiegato al capitolo 5 del report 
	- `AoAProbabilityPlot.m`: visualizzazione della distribuzione di angoli di attacco lungo tutta la traiettoria
3. _Optimization cartella contenente tutte le funzioni fitness utilizzate e citate nel report
	- `StabilityCheck.m`: calcola l'area della banda di condizioni iniziali per cui il boomerang torna indietro, una maggiore spiegazione è presente nel report al capitolo 9
	- `SpotAera.m` : è la medesima funzione StabilityCheck, solo senza grafici e 
	più veloce.
	- `GA_spot`: funzione fitness che calcola l'area della banda di condizioni iniziali per una geometria di boomerang di cui vanno specificati 3 parametri, delineati al capitolo 9 (Lunghezza pala, angolo di freccia e rapporto tra lunghezza della pala e corda)
	- `GA_para_chi`: funzione fitness che calcola la traiettoria dati in input due parametri r0 e phi, e tre parametri fissati Chi, D e Theta descritti al capitolo 6
	- `GA_FiveParameter`: funzione fitness che calcola la traiettoria dati in input cinque parametri descritti al capitolo 6 (r0,theta,D,phi,Vs)
	- `GA_mass`: funzione fitness che calcola la traiettoria dati in input il parametro di progetto Dens e tre parametri fissati Chi, D e Theta descritti al capitolo 6
4. _Miscellanea cartelle contente funzioni varie utili 
	- `HandInital.m` Calcolo delle condizioni iniziali dati i 5 parametri descritti al capitolo 5 del report
	- `FinalReport.m` Viene data una descrizione della traiettoria eseguita
	- `CheckBoomInfo.m` Funzione che permette di controllare se la funzione struct BoomInfo è stata creata correttamente
	- `Blender.m` Viene salvata un file .mat utile per visualizzare la traiettoria su blender
5. _Mecc Cartella contente le funzioni riguardanti il modello dinamico
	- `quatToEuler.m` dai quaternioni agli angoli di Eulero
	- `quatToAtt.m` dai quaternioni alla matrice di attitude To
	- `EventsQUAT.m` funzione evento per capire se il boomerang è arrivato a terra o tornato indietro
	- `EventsAntiSheronQUAT.m` funzione evento per evitare la creazione di traiettorie con più anelli 	 chiusi
	- `Eul_Quat.m` dagli angoli di Eulero ai quaternioni
	- `EquationOfMotionsQuaternion_IND.m` equazioni di moto con velocità indotta
	- `EquationOfMotionsQuaternion.m` equazioni di moto senza velocità indotta
6. _Geometry cartella contenente le funzioni rigurdati il modello geometrico
	- `Rot_point.m` funzioni per ruotare un insime di punti rispetto ad un punto
	- `Rot.m` viene fornite la matrice di rotazione rispetto ad un asse ed un angolo
	- `Profile2d_Trans.m` transizione dei profili nella parte centrale
	- `line_plane_intersection.m`@AuthorExternal 
	- `Boom3DShapes.m` viene creata la geometria 3D del boomerang
	- `Boom3DShape_VALIDAZIONE_BOOMERANG` modifica con calettamento del boomerang per la validazione
	- `AerCenter.m` calcola la posizione del centro aerodinamico di un profilo
	- _3DSOlidGeneration  @AuthorExternal funzioni chiave per il calcolo delle proprietà di massa del 	 boomerang
7. _Aero cartella contente le funzioni riguardanti il modello geometrico
	- `Xfoil.m` @AuthorExternal avvia @XFOIL e ricava i risultati
	- `xfoil.exe` @Xfoil
	- `t.m` funzione raccordo tra estrapolante e cl alpha
	- `midspan.m` calcola il punto medio tra due punti
	- `f_polar_360.m` fornisce le curva cl,cd,cm tra -180 e 180 °
	- `VelInd_FAST2.m` calcolo della velocità indotta 
	- `AeroDynamic_FAST_IND.m` calcolo delle forze aerodinamiche sfruttando le matrici 3D con la    velocità indotta
	- `AeroDynamic_FAST.m` calcolo delle forze aerodinamiche sfruttando le matrici 3D
	- `Profile_comparison_plot.m` plot cl_alpha e polari per il confronto
8. _Profile cartella contente vari profili da testare, per utilizzarli vanno NECESSARIAMENTE messi nella  cartella _Aero 
### Main:

1. _ValidationSolidworks.m confronto tra solidworks e Xboom con convergenza della soluzione
2. _Main_GA_Five_Parameter.m ottimizzazione delle condizioni iniziali per trovare un boomerang che torni indietro 
3. _Main_Example.m Simulazione della traiettoria di un boomerang avente come profilo alare un naca4412, serve per prendere atto del funzionamento del codice
4. _ConvergenzaForzeAerodinamiche.m 
5. _Cap9_Ottimizzazione.m procedura di ottimizzazione descritta al capitolo 9

## Blender

Permette di visualizzare la traiettoria su Blender
Procedura: 
Aprire Traiectory.blend
1. _T.mat salvare questo file utilizzando la funzione matlab `Blender.m`
2. Importare la geometria del boomerang su Blender tramite file->import->stl
3. andare su ->scripting e modificare la riga 8:
	- mat = scipy.io.loadmat('C:\\Users\\ciuti\\Documents\\GitHub\\XBoom\\Matlab_Code\\T.mat')
	inserire la posizione del file T.mat
	modificare la rifa 28:
	- Boom=bpy.context.scene.objects["Boom_D60_B0.5_P0"] inserire il nome della geometria
4. Opzionali modificando la riga 32 (togliendo il commento) e mettendo 'range(np.size(Time)):' alla riga 	30 è possibile avere una simulazione in cui il tempo coincide con quello reale 
## SolidWorks

SolidWorks è stato utilizzato per la stampa tridimensionale e per la verifica del calcolo delle proprietà di massa di XBOOM-Project
1. _Boom_D30_B0_P0.SLDPRT / _Stampa3D_ottimizzazione.SLDPRT : esempi di boomerang su solidworks
2. _Export_toSW.m: Vengono creati N file txt nella cartella ProfileSW, che successivamente possono essere importate su SolidWorks tramite la funzione CURVExyz di SolidWorks

## Validazione

Video delle traiettorie testate e post-processing per il confronto con i risultati simulati
1. Plot_confronto_validazione post-processing e confronto dei dati simulati e sperimentali
2. VideoValidazione video dei lanci del boomerang effettuati durante i tests

## Letteratura

Riferimenti utilizzati nel Report Finale
