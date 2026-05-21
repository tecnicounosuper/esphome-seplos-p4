void _SeplosV3::parse_buffer_() {
  if (rx_buffer_.size() < 5) return;

  // Cerchiamo un pacchetto che inizi col NOSTRO indirizzo specifico
  for (size_t i = 0; i <= rx_buffer_.size() - 5; i++) {
    
    // Controlla il byte di start: deve essere il MIO indirizzo e la funzione 0x04
    if (rx_buffer_[i] == this->address_ && rx_buffer_[i+1] == 0x04) {
      uint8_t byte_count = rx_buffer_[i+2];
      size_t frame_len = 3 + byte_count + 2;

      // Se non abbiamo ancora tutti i byte, usciamo e aspettiamo il prossimo loop
      if (rx_buffer_.size() < i + frame_len) return;

      // Se abbiamo trovato il NOSTRO pacchetto, elaboriamolo
      const uint8_t *data = &rx_buffer_[i + 3];

      // [Logica di estrazione invariata rispetto alla precedente]
      // ... (estrazione celle, temp, corrente, tensione, soc) ...

      // IMPORTANTE: Rimuoviamo dal buffer solo il pacchetto che abbiamo appena consumato
      // Questo NON tocca i dati dell'altro BMS che potrebbero essere nel buffer
      rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + i + frame_len);
      return; 
    }
  }
  
  // Se abbiamo scansionato tutto e non abbiamo trovato il NOSTRO indirizzo,
  // ma il buffer è troppo pieno, puliamo i byte "orfani" (spazzatura)
  if (rx_buffer_.size() > 128) {
      rx_buffer_.clear(); 
  }
}
