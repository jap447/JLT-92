"""Shared CLI options and visible attribution for homework plots."""
import argparse
import os
from pathlib import Path
import re


def parse_netids(value):
    netids = tuple(part.strip().lower() for part in value.split(','))
    if not netids or any(not re.fullmatch(r'[a-z][a-z0-9]*', item) for item in netids):
        raise argparse.ArgumentTypeError('Use comma-separated NetIDs, e.g. abc123,xyz789 (no spaces).')
    return tuple(dict.fromkeys(netids))


def add_plot_arguments(parser):
    parser.add_argument('--netids', type=parse_netids, required=True,
                        metavar='NETID[,NETID...]',
                        help='NetIDs to display on the plot; list every group member')


def add_export_arguments(parser, rrt=False):
    parser.add_argument('--save-image', metavar='FILE.png',
                        help='Save the final plot, e.g. writeup/astar.png')
    parser.add_argument('--no-show', action='store_true',
                        help='Run without opening a plot window')
    if rrt:
        parser.add_argument('--save-gif', metavar='FILE.gif',
                            help='Save tree growth, e.g. writeup/rrt.gif')


def prepare_output_path(filename, suffix):
    path = Path(filename).expanduser()
    if path.suffix.lower() != suffix:
        raise ValueError('Output filename must end in {}: {}'.format(suffix, path))
    if path.is_dir():
        raise ValueError('Output filename is a directory: {}'.format(path))
    path.parent.mkdir(parents=True, exist_ok=True)
    if not os.access(str(path.parent), os.W_OK) or (path.exists() and not os.access(str(path), os.W_OK)):
        raise ValueError('Output location is not writable: {}'.format(path))
    return path


def configure_exports(args, parser):
    """Call before importing planner modules, which themselves import pyplot."""
    try:
        for name, suffix in (('save_image', '.png'), ('save_gif', '.gif')):
            value = getattr(args, name, None)
            if value is not None:
                setattr(args, name, prepare_output_path(value, suffix))
    except (OSError, ValueError) as error:
        parser.error(str(error))
    if args.no_show:
        import matplotlib
        matplotlib.use('Agg')


def label_plot(figure, netids):
    """Label the figure itself so screenshots and saved images include attribution."""
    if netids:
        figure.suptitle('NetIDs: ' + ', '.join(netids), fontsize=11)
        figure.subplots_adjust(top=0.90)
