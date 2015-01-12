Ts_ADC1 = 1e-6; % Sample time of ADC1 [s] (All channels ADC_SAMPLE_1P5)
ADC_COMMUTATE_NUM_CHANNELS = 1;
ADC_COMMUTATE_BUF_DEPTH = 50;
f_single = 1 / (Ts_ADC1 * ADC_COMMUTATE_NUM_CHANNELS) % Sample frequency of each channel
T_cb_ADC1 = Ts_ADC1 * ADC_COMMUTATE_NUM_CHANNELS * ADC_COMMUTATE_BUF_DEPTH / 2 % Period of ADC1 streaming callback
f_cb_ADC1 = 1 / T_cb_ADC1

f_PWM = 20e3;
duty_cycle = 0.6;
t_on = duty_cycle * 1/f_PWM
samples_when_on = t_on * f_single


% Check max samples for linear fit;
x = 0;
sumx2 = 0;
sumy = 0;
sumy2 = 0;
NREG = 20;
while sumy * sumx2 < 2^63
  x = x + 1;
  sumx2 = sumx2 + x^2;
  if x <= NREG
    sumy = sumy + 1000;
    sumy2 = sumy2 + 1000^2;
  end
end
disp('Maximum number of samples')
x
sumy
sumy2





