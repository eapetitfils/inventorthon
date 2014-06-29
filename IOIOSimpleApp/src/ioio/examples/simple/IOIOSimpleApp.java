package ioio.examples.simple;

import java.util.Random;

import ioio.lib.api.AnalogInput;
import ioio.lib.api.DigitalOutput;
import ioio.lib.api.IOIO;
import ioio.lib.api.IOIOFactory;
import ioio.lib.api.PwmOutput;
import ioio.lib.api.TwiMaster;
import ioio.lib.api.exception.ConnectionLostException;
import ioio.lib.util.BaseIOIOLooper;
import ioio.lib.util.IOIOLooper;
import ioio.lib.util.android.IOIOActivity;
import android.os.Bundle;
import android.view.View;
import android.view.View.OnClickListener;
import android.widget.Button;
import android.widget.SeekBar;
import android.widget.TextView;
import android.widget.ToggleButton;

public class IOIOSimpleApp extends IOIOActivity implements OnClickListener {
	private TextView textViewInput_;
	private TextView textViewOutput_;
	private Button mLed1Button, mLed2Button, mLed3Button;
	private boolean checked_ = true;
	private int mode = 0;
	
	@Override
	public void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		setContentView(R.layout.main);
		textViewInput_ = (TextView) findViewById(R.id.textViewInput);
		textViewOutput_ = (TextView) findViewById(R.id.textViewOutput);
		mLed1Button = (Button) findViewById(R.id.button1);
		mLed1Button.setOnClickListener(this);
		mLed2Button = (Button) findViewById(R.id.button2);
		mLed2Button.setOnClickListener(this);
		
		enableUi(false);
	}

	class Looper extends BaseIOIOLooper {
		private AnalogInput input_;
		private DigitalOutput led_;
		private TwiMaster twi;
		private int address = 0;
		byte[] request = new byte[] { 0x01 };
		byte[] response = new byte[4];

		@Override
		public void setup() throws ConnectionLostException {
			led_ = ioio_.openDigitalOutput(IOIO.LED_PIN, true);
			input_ = ioio_.openAnalogInput(40);
			enableUi(true);
		}

		@Override
		public void loop() throws ConnectionLostException, InterruptedException {
	//		setNumber(input_.read());
	//		twi.writeRead(value, false, request, request.length, response, response.length);
			if (mode == 1) {
				twi = ioio_.openTwiMaster(address, TwiMaster.Rate.RATE_100KHz, false);
				twi.writeReadAsync(address, false, request, request.length, response, response.length);
				twi.close();
				mode = 0;
				checkTWI(request,response);
			}
			if (mode == 2) {
				twi = ioio_.openTwiMaster(address, TwiMaster.Rate.RATE_100KHz, false);
				twi.writeReadAsync(address, false, request, request.length, response, response.length);
				twi.close();
				checkTWI(request,response);
			}
			
			led_.write(checked_);
			
			Thread.sleep(100);
		}

		@Override
		public void disconnected() {
			enableUi(false);
		}
	}

	@Override
	protected IOIOLooper createIOIOLooper() {
		return new Looper();
	}

	private void enableUi(final boolean enable) {
		runOnUiThread(new Runnable() {
			@Override
			public void run() {

			}
		});
	}
	
	private void checkTWI(byte[] request, byte[] response) {
		final String r1 = new String(request);
		final String r2 = new String(response);
		final int sr1 = request.length;
		final int sr2 = response.length;
		runOnUiThread(new Runnable() {
			@Override
			public void run() {
				textViewOutput_.setText(r1.concat(" of size ").concat(String.valueOf(sr1)));
				textViewInput_.setText(r2.concat(" of size ").concat(String.valueOf(sr2)));
				if (mode > 0){
					checked_ = !checked_;
				}
			}
		});
	}
	
	@Override
	public void onClick(View v) {
		switch (v.getId()) {
		case R.id.button1:
			mode = 1;
			checked_ = !checked_;
			break;

		case R.id.button2:
			if (mode == 2){
				mode = 0;
				mLed2Button = (Button) findViewById(R.id.button2);
				mLed2Button.setText("I2C Frenzy!");
			}
			else
			{
				mode = 2;
				mLed2Button = (Button) findViewById(R.id.button2);
				mLed2Button.setText("Stop this Frenzy!");
			}
			break;

		default:
			break;
		}
	}
}