import numpy as np
import matplotlib.pyplot as plt
import matplotlib
# matplotlib.use('pgf')

def set_font(size):
    matplotlib.rcParams.update({# Use mathtext, not LaTeX
                            'text.usetex': False,
                            # Use the Computer modern font
                            'font.family': 'serif',
                            'font.serif': ['cmr10'],
                            'font.size' : size,
                            'mathtext.fontset': 'cm',
                            # Use ASCII minus
                            'axes.unicode_minus': False,
                            })

    # matplotlib.rcParams.update({# Use mathtext, not LaTeX
    #                         'text.usetex': False,
    #                         # Use the Computer modern font
    #                         'font.family': 'serif',
    #                         'font.serif': ['Linux Libertine O'],
    #                         'font.size' : size,
    #                         'mathtext.fontset': 'cm',
    #                         # Use ASCII minus
    #                         'axes.unicode_minus': False,
    #                         'text.latex.preamble': [
    #     r"\usepackage{libertine}"
    #     r"\usepackage{libertinust1math}"
    #     r"\usepackage[T1]{fontenc}",
    #  ]
    #                         })

    # print("SET FONT")
    # matplotlib.rcParams.update({# Use mathtext, not LaTeX
    #                         'text.usetex': True,
    #                         # Use the Computer modern font
    #                         # 'font.family': 'serif',
    #                         'font.serif': 'Computer Modern Roman',
    #                         # 'text.latex.unicode': True,
    #                         'font.size' : size,
    #                         'mathtext.fontset': 'cm',
    #                         # Use ASCII minus
    #                         'axes.unicode_minus': False,
    #                         })
linewidth = 0.5
minor_tick_color = (0.9, 0.9, 0.9)
kFontSize = 6

legend_handles = []
legend_labels = []

def setupfig(current_fig=None, halfsize=False, thirdsize=False, quartersize=False):
    global legend_handles
    global legend_labels
    legend_handles = []
    legend_labels = []
    if current_fig is None:
        plt.clf()
        fig = plt.gcf()
    else:
        fig = current_fig
    set_font(kFontSize)
    kScaleDown = 2.1
    kVerticalScale = 0.9
    halfsize_scale = 1
    if halfsize:
        halfsize_scale = 0.45
    if thirdsize:
        halfsize_scale = 0.30
    if quartersize:
        halfsize_scale = 0.19
    fig.set_size_inches(7 /kScaleDown * 1.58 * halfsize_scale , 7.8 / kScaleDown / 1.61 * kVerticalScale, forward=True)
    plt.gca().set_axisbelow(True)

def grid(plt=plt):
    plt.grid(linewidth=linewidth/2)
    plt.grid(which='minor', color=minor_tick_color, linestyle='--', alpha=0.7, clip_on=True, linewidth=linewidth/4)

def color(count, total_elements):
    start = 0.2
    stop = 0.8
    cm_subsection = np.linspace(start, stop, total_elements) 
    return [ matplotlib.cm.magma(x) for x in cm_subsection ][count]

def alpha(color, val=0.5):
    assert(type(color) == tuple)
    assert(len(color) == 4)
    r, g, b, a=color
    return (r, g, b, val)

def add_legend(handle, label):
    global legend_handles
    global legend_labels
    legend_handles.append(handle)
    legend_labels.append(label)

def legend(loc, plt=plt):
    if type(loc)  is str:
        if loc == 'ul':
            loc = 2
        elif loc == 'br':
            loc = 4
        else:
            assert(False)
    handles, labels = plt.gca().get_legend_handles_labels()
    leg = plt.legend(handles + legend_handles, labels + legend_labels, loc=loc, prop={'size': 4})
    # set the linewidth of each legend object
    for legobj in leg.legendHandles:
        legobj.set_linewidth(linewidth * 3)

def save_fig(filename):
    print("Saving figout/{}.*".format(filename))
    # plt.savefig("figout/{}.pgf".format(filename), bbox_inches='tight', pad_inches=0)
    # plt.savefig("figout/{}.png".format(filename), bbox_inches='tight', dpi=400, transparent=True, pad_inches=0)
    plt.savefig("figout/{}.pdf".format(filename), bbox_inches='tight', pad_inches=0)

