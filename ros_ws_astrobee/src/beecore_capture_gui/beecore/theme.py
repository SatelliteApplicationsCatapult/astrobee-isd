"""Brand palette and global styling.

Source colours (Satellite Applications Catapult):
    PANTONE 485 C                    #E32119
    PANTONE Process Black            #231F20   surfaces
    PANTONE 2665 "Beyond Our Earth"  #7E57C5   banner + debug accent

The logo artwork measures #EE3023, marginally brighter than 485 C. The palette
keeps 485 C for UI accents; LOGO_RED records the artwork value.

NOTE: this uses string.Template, not %-formatting. CSS is full of percent
signs - "100%", "@keyframes 0%,100%" - and every one of them has to be doubled
under %-formatting, including any that appear inside comments. Template uses
$name and leaves % alone, so that whole class of bug cannot happen here.
"""

from string import Template

from nicegui import ui

# --- brand -------------------------------------------------------------------

RED = '#E32119'          # PANTONE 485 C
LOGO_RED = '#EE3023'     # measured from SA_SM_Colour.png
BLACK = '#231F20'        # PANTONE Process Black
VIOLET = '#7E57C5'       # PANTONE 2665 - banner

# --- derived -----------------------------------------------------------------

VIOLET_DIM = '#5D3F94'   # debug bar, related to the banner but distinct
SURFACE = '#2C2728'
SURFACE_HI = '#3A3435'
BORDER = '#4A4243'
TEXT = '#F4F1F1'
MUTED = '#A49E9F'
LOG_BG = '#191617'

# --- semantic ----------------------------------------------------------------

GREEN = '#2E9E5B'        # ready / go
AMBER = '#D98A1F'        # stale / warning
GREY = '#6B6465'         # unknown


_CSS = Template('''
    body, .nicegui-content { background: $black; color: $text; }

    .q-card { background: $surface !important;
              border: 1px solid $border; box-shadow: none; }

    .eyebrow { font-size: 0.68rem; letter-spacing: 0.14em;
               text-transform: uppercase; color: $muted; }

    .q-tabs { background: $surface; border-bottom: 1px solid $border; }
    .q-tab { color: $muted; }
    .q-tab--active { color: $text; }
    /* The one place 485 C earns its keep: a red underline on the active tab,
       reading against the violet banner without competing with it. */
    .q-tab__indicator { background: $red !important; height: 3px; }
    .q-tab-panels, .q-tab-panel { background: $black !important; }

    /* Flex children default to min-height:auto, so tall tab content grows the
       page instead of scrolling inside it - which left only the banner visible
       at full browser zoom. */
    .q-splitter, .q-splitter__panel,
    .q-splitter__before, .q-splitter__after { min-height: 0; }
    .q-tab-panels { flex: 1 1 0; min-height: 0; overflow: hidden; }
    .q-tab-panel { height: 100%; overflow-y: auto; overflow-x: hidden; }
    .debug-pane { min-height: 0; }
    .nicegui-log { min-height: 0; }

    .debug-bar { background: $violet_dim; }
    .debug-pane { background: $log_bg; border-top: 2px solid $violet; }

    .q-splitter__separator { background: $border; }
    .q-splitter__separator:hover { background: $violet; }

    .q-field--outlined .q-field__control { background: $surface_hi; }

    /* Diagnostic LEDs */
    .led { width: 13px; height: 13px; border-radius: 50%;
           display: inline-block; flex: 0 0 auto;
           box-shadow: 0 0 6px currentColor; }

    @keyframes recpulse { 0%,100% { opacity: 1 } 50% { opacity: 0.35 } }
    .rec-live { animation: recpulse 1.4s ease-in-out infinite; }
    @media (prefers-reduced-motion: reduce) { .rec-live { animation: none; } }

    :focus-visible { outline: 2px solid $violet; outline-offset: 2px; }
''')


def apply_theme() -> None:
    ui.colors(primary=VIOLET, secondary=VIOLET, accent=VIOLET,
              dark=BLACK, positive=GREEN, negative=RED, warning=AMBER)
    ui.add_css(_CSS.substitute(
        red=RED, black=BLACK, violet=VIOLET, violet_dim=VIOLET_DIM,
        surface=SURFACE, surface_hi=SURFACE_HI, border=BORDER,
        text=TEXT, muted=MUTED, log_bg=LOG_BG))
