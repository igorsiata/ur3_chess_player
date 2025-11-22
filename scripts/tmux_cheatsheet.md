# TMUX Cheat Sheet

## Prefix
`Ctrl + b` — podstawowy prefix do wszystkich komend tmux

---

## Windows (zakładki)
| Akcja | Skrót |
|-------|-------|
| Nowa zakładka | `Ctrl+b` then `c` |
| Następna zakładka | `Ctrl+b` then `n` |
| Poprzednia zakładka | `Ctrl+b` then `p` |
| Przejdź do zakładki nr X | `Ctrl+b` then `<number>` |
| Lista zakładek | `Ctrl+b` then `w` |
| Zmień nazwę zakładki | `Ctrl+b` then `,` |

---

## Panes (podziały ekranu)
| Akcja | Skrót |
|-------|-------|
| Podział pionowy | `Ctrl+b` then `%` |
| Podział poziomy | `Ctrl+b` then `"` |
| Przejdź do następnego panelu | `Ctrl+b` then `o` |
| Przejdź w konkretnym kierunku | `Ctrl+b` then `Arrow keys` |
| Zmiana rozmiaru panelu | `Ctrl+b` then `Ctrl+Arrow` |

---

## Scroll / Copy Mode
| Akcja | Skrót |
|-------|-------|
| Wejście w tryb scroll | `Ctrl+b` then `[` |
| Poruszanie się | Arrow keys / PageUp / PageDown |
| Wyjście z trybu scroll | `q` lub `Enter` |

---

## Sesje
| Akcja | Skrót / Komenda |
|-------|----------------|
| Odłącz sesję (detach) | `Ctrl+b` then `d` |
| Lista sesji | `tmux ls` |
| Dołącz do sesji | `tmux attach -t <session_name>` |
| Zabij sesję | `tmux kill-session -t <session_name>` |
| Zabij wszystkie sesje | `tmux kill-server` |

---

## Inne przydatne
| Akcja | Skrót |
|-------|-------|
| Zabij bieżący panel | `Ctrl+b` then `x` |
| Odśwież konfigurację | `tmux source-file ~/.tmux.conf` |
| Zmień indeks początkowy okien | `set -g base-index 1` (w `.tmux.conf`) |
| Zmień indeks początkowy paneli | `set -g pane-base-index 1` (w `.tmux.conf`) |
