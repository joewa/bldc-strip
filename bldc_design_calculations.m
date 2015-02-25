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

disp('Schaltzeiten')
PWM_CLOCK_FREQUENCY = 28e6;
PWM_DEFAULT_FREQUENCY = 100000; % [40e3, 50e3, 62500, 100e3]	choose one of these base frequencies [Hz]
PWM_MINIMUM_FREQUENCY = 40000;

ADC_COMMUTATE_FREQUENCY	= 1e6;%		// [Hz]
ADC_PWM_DIVIDER = (PWM_CLOCK_FREQUENCY / ADC_COMMUTATE_FREQUENCY);
ADC_PWM_PERIOD = (ADC_COMMUTATE_FREQUENCY / PWM_DEFAULT_FREQUENCY);
PWM_MAXIMUM_PERIOD = (ADC_PWM_DIVIDER * ADC_COMMUTATE_FREQUENCY / PWM_MINIMUM_FREQUENCY);

d_percent = (1:49) + 49;
pwm_period = fix( ((ADC_PWM_PERIOD * 2500) ./ ((100 .- d_percent) .* d_percent))) .* ADC_PWM_DIVIDER;

for i=1:length(pwm_period)
  if pwm_period(i) > PWM_MAXIMUM_PERIOD
    pwm_period(i) = PWM_MAXIMUM_PERIOD;
  endif
endfor

t_on = d_percent ./ 100 .* pwm_period;
t_off = pwm_period - t_on;
plot(d_percent, pwm_period, d_percent, t_off)
% plot(d_percent, t_off)
% Check max samples for linear fit;





