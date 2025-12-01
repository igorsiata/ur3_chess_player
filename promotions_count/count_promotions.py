# stream_pgn_zst_promotions.py
# Efficiently parse .pgn.zst files and count promotions.
# Requires: pip install zstandard python-chess

import sys
import zstandard as zstd
import chess.pgn
from collections import Counter
import io

def parse_zst_pgn(path):
    """Stream games from a .pgn.zst file without fully decompressing."""
    with open(path, 'rb') as fh:
        dctx = zstd.ZstdDecompressor(max_window_size=2**31)
        with dctx.stream_reader(fh) as reader:
            # python-chess requires a text stream, so wrap it:
            text_stream = io.TextIOWrapper(reader, encoding='utf-8', errors='replace')
            while True:
                game = chess.pgn.read_game(text_stream)
                if game is None:
                    break
                yield game


def count_promotions_in_game(game):
    """Return Counter of promotions in a single game."""
    board = game.board()
    c = Counter()
    node = game

    while node.variations:
        move = node.variation(0).move
        if move.promotion:
            piece = chess.PIECE_SYMBOLS[move.promotion].upper()  # 'Q','N','R','B'
            c[piece] += 1
        board.push(move)
        node = node.variation(0)

    return c


def main(path):
    total_games = 0
    games_with_prom = 0
    promotion_counts = Counter()

    for game in parse_zst_pgn(path):
        total_games += 1

        c = count_promotions_in_game(game)
        if sum(c.values()) > 0:
            games_with_prom += 1

        promotion_counts.update(c)

        if total_games % 10000 == 0:
            print(f"Processed {total_games:,} games...", flush=True)

    total_prom = sum(promotion_counts.values())

    print("\n=== Results ===")
    print(f"Total games: {total_games:,}")
    print(f"Games with ≥1 promotion: {games_with_prom:,} ({games_with_prom/total_games:.4%})")
    print(f"Total promotions: {total_prom:,}")
    for p in ["Q", "N", "R", "B"]:
        count = promotion_counts[p]
        pct = count / total_prom if total_prom else 0
        print(f"  {p}: {count:,} ({pct:.4%})")


if __name__ == "__main__":
    main("/home/igorsiata/ur3_chess_player/lichess_db_standard_rated_2015-01.pgn.zst")


#=== Results ===
# Total games: 1,497,237
# Games with ≥1 promotion: 186,299 (12.4429%)
# Total promotions: 235,303
#   Q: 226,408 (96.2198%)
#   N: 2,048 (0.8704%)
#   R: 5,732 (2.4360%)
#   B: 1,115 (0.4739%)
