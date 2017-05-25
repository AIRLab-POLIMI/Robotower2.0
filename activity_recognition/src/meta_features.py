import numpy as np
import pandas as pd
import scipy.fftpack as fft
from scipy.stats import norm, skew, kurtosis,entropy
from detect_peaks import detect_peaks


def mean(data):
    """Calculates mean of the data"""
    return np.mean(data)

def std(data):
    """Calculates the standard deviation"""
    return np.std(data)

def max_min(data):
    """Calculate the max - min."""
    return np.max(data) - np.min(data)

def max_value(data):
    """ Calculates Largest value in array"""
    return np.max(data)
    
def min_value(data):
    """Calculates smallest value in array"""
    return np.min(data)

def mad(data, axis=None):
    """ 
    Calculates the median absolute deviation: a "Robust" version of standard deviation.
        Indices variabililty of the sample.
        https://en.wikipedia.org/wiki/Median_absolute_deviation 
    """
    return np.median(np.absolute(data - np.median(data, axis)), axis)


def sma(data):
    """Computes Signal magnitude area.
    http://uclab.khu.ac.kr/resources/publication/J_99.pdf
    """
    return np.sum([np.abs(x) for x in data])


def energy(data):
    """Energy measure. Sum of the squares divided by the number of values."""
    return np.sum([x**2 for x in data]) / float(len(data))


def iqr(data):
    """Calculates the interquartile range
    http://stackoverflow.com/questions/23228244/how-do-you-find-the-iqr-in-numpy
    """
    return np.subtract(*np.percentile(data, [75, 25]))

def maxInds(data, n_bins=200):
    """Returns the index of the frequency component with largest magnitude"""
    
    mean_sig = np.ones_like(data)*np.mean(data)
    # remove mean of the signal, for better results.
    sig = data - mean_sig
    freqsig = fft.fft(sig,n=n_bins) 
    half_freq_domain = freqsig[:int(n_bins/2)]
    #get max index in the freq domain
    return np.where(np.abs(half_freq_domain)==(max(np.abs(half_freq_domain))))[0][0]

def meanFreq(data, n_bins=200):
    """
    Weighted average of the frequency components to obtain a mean frequency
    http://luscinia.sourceforge.net/page26/page35/page35.html
    """
    mean_sig = np.ones_like(data)*np.mean(data)
    # remove mean of the signal, for better results.
    sig = data - mean_sig
    freqsig = fft.fft(sig,n=n_bins) 
    half_freq_domain = freqsig[:int(n_bins/2)]
    return np.sum(np.abs(half_freq_domain) * range(len(half_freq_domain))) / sum(np.abs(half_freq_domain))

def skewness(data):
    return skew(data)

def kurtos(data):
    return kurtosis(data)

def freq_skewness(data, n_bins=200): 
    """skewness of the frequency domain signal"""
    mean_sig = np.ones_like(data)*np.mean(data)
    # remove mean of the signal, for better results.
    sig = data - mean_sig
    freqsig = fft.fft(sig,n=n_bins) 
    half_freq_domain = freqsig[:int(n_bins/2)]
    return skew(np.abs(half_freq_domain))

def freq_kurtos(data, n_bins=200):
    """kurtosis of the frequency domain signal"""
    mean_sig = np.ones_like(data)*np.mean(data)
    # remove mean of the signal, for better results.
    sig = data - mean_sig
    freqsig = fft.fft(sig,n=n_bins) 
    half_freq_domain = freqsig[:int(n_bins/2)]
    return kurtosis(np.abs(half_freq_domain))

def fft_energy(data,n_bins=50):
    n_bins = len(data)
    #mean_sig = np.ones_like(data)*np.mean(data)
    # remove mean of the signal, for better results.
    sig = data #- mean_sig
    freqsig = fft.fft(sig,n=n_bins) 
    half_freq_domain = freqsig[:int(n_bins/2)]
    #return [np.abs(x)**2 for x in half_freq_domain]
    return np.sum([np.abs(x)**2 for x in freqsig]) / float(len(freqsig))

def pse(data, n_bins=200):
    """extraction of power spectral entropy
    http://stackoverflow.com/questions/30418391/what-is-frequency-domain-entropy-in-fft-result-and-how-to-calculate-it"""
    #mean_sig = np.ones_like(data)*np.mean(data)
    # remove mean of the signal, for better results.
    sig = data #- mean_sig
    freqsig = fft.fft(sig,n=n_bins) 
    half_freq_domain = freqsig[:int(n_bins/2)]
    norm_const = len(half_freq_domain)
    calc_pse = [np.abs(x)**2/norm_const for x in half_freq_domain]
    sum_pse = np.sum(calc_pse)
    n_pse = [v/sum_pse for v in calc_pse]
    return entropy(n_pse)

def num_peaks(data):
    """Return the number of peaks found on the data"""
    return len(detect_peaks(data, mpd=10, show=False))

def moving_rmsV1(x, window):
    """Moving RMS of 'x' with window size 'window'."""
    window = 2*window + 1
    return np.sqrt(np.convolve(x*x, np.ones(window)/window, 'same'))

def rms(data):
    """Return the root mean square of the sequence"""
    return np.sum(np.square(data))/len(data)

def correlation(t1,t2): 
    """correlation coefficient between two accelerometer signals"""
    data = {'data1':t1,'data2':t2}
    cor = pd.DataFrame.from_dict(data).corr().to_dict()['data1']['data2']
    return cor