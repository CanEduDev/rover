import csv
import argparse

import can
import cantools


def main():
    parser = argparse.ArgumentParser(
        description="Decode CAN log using DBC and export to CSV"
    )
    parser.add_argument("dbc_file", help="Path to the DBC file")
    parser.add_argument("log_file", help="Path to the CAN log file (ASC, BLF, etc.)")
    parser.add_argument("output_csv", help="Path to output CSV file")
    args = parser.parse_args()

    db = cantools.db.load_file(args.dbc_file)
    log = can.LogReader(args.log_file)

    with open(args.output_csv, mode="w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["Timestamp", "Message Name", "ID", "Signal", "Value"])

        for msg in log:
            if msg.is_error_frame or msg.is_remote_frame:
                continue
            try:
                decoded = db.decode_message(msg.arbitration_id, msg.data)  # type: ignore
                message_name = db.get_message_by_frame_id(msg.arbitration_id).name  # type: ignore
                for signal, value in decoded.items():  # type: ignore
                    writer.writerow(
                        [
                            msg.timestamp,
                            message_name,
                            hex(msg.arbitration_id),
                            signal,
                            value,
                        ]
                    )
            except Exception:  # Skip messages not listed in DB
                continue


if __name__ == "__main__":
    main()
