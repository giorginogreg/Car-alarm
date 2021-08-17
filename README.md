# Funzionamento

## Moduli installati
Sensore Ultrasuoni  
Sim808 ( GSM + GPS )  
Gyroscope  
Arduino

## Casi d'uso
- Botta all'auto - Spostamento con gyroscopio  
- Furto durante la guida  
- Furto nella notte -> sensore ultrasuoni
- Furto con jammer -> sensore ultrasuoni + detection jamming + allarme locale (tromba)

## Comportamento allarme
<pre>
Msg per far partire tracking  ->
    Attivo rete internet  
    Attivo GPS  
    Ogni tot viene inviata posizione a server  
</pre>
<pre>
Msg per stoppare tracking  
    Spengo rete internet
</pre>
Stato Pin rosa <-> stato ultrasuoni e gyroscopio

Prevedere modalità per collegare gsm in caso di non collegamento, e se necessario gps.

# Heroku server
Previsto server con script php che riceve chiamate post con parametri coordinate, e salva nel db le coord.
In get vengono selezionati in base ai giorni le coord e i tracciati della macchina.

---
Da fare  
 - inserire chiamata se qualcosa non si setta correttamente
 - Invio richieste get/post a pagina che determina possibile antijamming se non riceve heartbeat per tot minuti