from .app import main

try:
    main()
except (ValueError, OSError) as exc:
    raise SystemExit('face-table: ' + str(exc)) from None
