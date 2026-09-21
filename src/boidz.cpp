#include "plugin.hpp"

/*
Oh god, don't look here, my spaghetti... it's everywhere
I swear I can code...
*/

struct Boidz : Module {
	enum ParamId {
		NUMBOIDS_PARAM,
		COHESIONATTN_PARAM,
		ALIGNMENTATTN_PARAM,
		SEPERATIONATTN_PARAM,
		PARAMS_LEN
	};
	enum InputId {
		RESET_INPUT,
		COHESIONIN_INPUT,
		ALIGNMENTIN_INPUT,
		SEPERATIONIN_INPUT,
		INPUTS_LEN
	};
	enum OutputId {
		TRIGA1_OUTPUT,
		TRIGA2_OUTPUT,
		TRIGA3_OUTPUT,
		TRIGB1_OUTPUT,
		TRIGB2_OUTPUT,
		TRIGB3_OUTPUT,
		TRIGC1_OUTPUT,
		TRIGC2_OUTPUT,
		TRIGC3_OUTPUT,
		TRIGALL_OUTPUT,
		OUTPUTS_LEN
	};
	enum LightId {
		LIGHTS_LEN
	};

	static const int MAX_UNITS = 16;
	static constexpr float MAXCENTERDIST = 30.f;

	struct Unit{
		Vec position = Vec(0.f, 0.f);
		Vec direction = Vec(0.f, 1.f);
		Vec nextDirection = Vec(0.f, 1.f);
		bool visible = false;
		int occupiedSpace = 0;

		Unit(){
			this->position = Vec((rand() % (int)(MAXCENTERDIST * 2)) - MAXCENTERDIST, (rand() % (int)(MAXCENTERDIST * 2)) - MAXCENTERDIST);
			this->direction = Vec(0.f, 1.f).rotate(((rand() % 4) - 1) * (M_PI / 4.f));
		}

		bool updateOccupiedSpace(){
			int row = 0;
			int col = 0;

			int lastSpace  = occupiedSpace;

			if(position.x < -MAXCENTERDIST * (1.f/3.f)){
				row = 0;
			}
			else if(position.x > MAXCENTERDIST * (1.f/3.f)){
				row = 2;
			}
			else{
				row = 1;
			}

			if(-position.y > MAXCENTERDIST * (1.f/3.f)){
				col = 0;
			}
			else if(-position.y < -MAXCENTERDIST * (1.f/3.f)){
				col = 2;
			}
			else{
				col = 1;
			}

			occupiedSpace = (row) + (col * 3);

			return (occupiedSpace == lastSpace) ? false : true;
		}
	};

	const float UNITSPEED = 3.f;
	const float UNITTURNING = nvgDegToRad(270.f);
	const float PERSONALSPACE = 5.5f;

	Unit units[MAX_UNITS];
	int unitCount = 0;

	dsp::PulseGenerator gridPulseGens[10];
	dsp::SchmittTrigger resetTrigger = dsp::SchmittTrigger();

	Boidz() {
		config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
		configParam(NUMBOIDS_PARAM, 1.f, MAX_UNITS, MAX_UNITS / 2.f, "Number of Boids");
		configParam(COHESIONATTN_PARAM, 0.f, 1.f, 0.5f, "Cohesion %");
		configParam(ALIGNMENTATTN_PARAM, 0.f, 1.f, 0.5f, "Alignment %");
		configParam(SEPERATIONATTN_PARAM, 0.f, 1.f, 0.5f, "Seperation %");
		configInput(RESET_INPUT, "Reset");
		configInput(COHESIONIN_INPUT, "Cohesion C/V");
		configInput(ALIGNMENTIN_INPUT, "Alignment C/V");
		configInput(SEPERATIONIN_INPUT, "Seperation C/V");
		configOutput(TRIGA1_OUTPUT, "A1 Trig Out");
		configOutput(TRIGA2_OUTPUT, "A2 Trig Out");
		configOutput(TRIGA3_OUTPUT, "A3 Trig Out");
		configOutput(TRIGB1_OUTPUT, "B1 Trig Out");
		configOutput(TRIGB2_OUTPUT, "B2 Trig Out");
		configOutput(TRIGB3_OUTPUT, "B3 Trig Out");
		configOutput(TRIGC1_OUTPUT, "C1 Trig Out");
		configOutput(TRIGC2_OUTPUT, "C2 Trig Out");
		configOutput(TRIGC3_OUTPUT, "C3 Trig Out");
		configOutput(TRIGALL_OUTPUT, "ALL Trig Out");

		for(int i = 0; i < MAX_UNITS; i++){
			units[i] = Unit();
			units[i].visible = true;
		}
		for(int i = 0; i < 10; i++){
			gridPulseGens[i] = dsp::PulseGenerator();
		}
	}

	void changeUnitCount(int newCount){
		for(int i = 0; i < MAX_UNITS; i++){
			units[i] = Unit();
			units[i].visible = i < newCount ? true : false;
		}
		
		unitCount = newCount;
	}

	void triggerSpace(int index){
		gridPulseGens[index].trigger();
		gridPulseGens[9].trigger();// Trigger all output as well.
	}

	void updateWeights(){
		seperation = params[SEPERATIONATTN_PARAM].getValue();
		if(inputs[SEPERATIONIN_INPUT].isConnected()) seperation *= inputs[SEPERATIONIN_INPUT].getVoltage() * 0.1f;
		alignment = params[ALIGNMENTATTN_PARAM].getValue();
		if(inputs[ALIGNMENTIN_INPUT].isConnected()) seperation *= inputs[ALIGNMENTIN_INPUT].getVoltage() * 0.1f;
		cohesion = params[COHESIONATTN_PARAM].getValue();
		if(inputs[COHESIONIN_INPUT].isConnected()) seperation *= inputs[COHESIONIN_INPUT].getVoltage() * 0.1f;
	}

	float seperation = 1.f;
	float alignment = 1.f;
	float cohesion = 1.f;

	void process(const ProcessArgs& args) override { // ######################### MAIN LOOP ############################
		const float dt = args.sampleTime;

		updateWeights();

		if(inputs[RESET_INPUT].isConnected() && resetTrigger.process(inputs[RESET_INPUT].getVoltage(), 0.1f, 1.f)){
			changeUnitCount(unitCount);
		}
		else{
			int numboids = (int)params[NUMBOIDS_PARAM].getValue();
			if(numboids != unitCount) changeUnitCount(numboids);
		}
		
		
		for(Unit& unit : units){
			if(unit.visible == false) continue;

			Vec avgHeading = Vec(0.f);
			Vec avgPosition = Vec(0.f);
			Vec closestPosition = Vec(0.f);
			int unitsCounted = 0;

			for(Unit targetUnit : units){

				Vec targetRelative =  (targetUnit.position - unit.position);

				if(targetUnit.visible == false || targetUnit.position == unit.position) continue;
				else if(targetRelative.norm() > PERSONALSPACE) continue;

				if(targetRelative.norm() < closestPosition.norm()){
					closestPosition = targetRelative;
				}

				avgPosition += targetRelative;
				avgHeading += (targetUnit.direction - unit.direction).normalize();
				unitsCounted++;
			}

			if(unitsCounted > 1){
				avgPosition /= unitsCounted + 1;
				avgHeading /= unitsCounted + 1;

				float alignWeight = sgn(unit.direction.dot(avgHeading.normalize())) * alignment;
				float cohesionWeight = sgn(unit.direction.rotate(M_PI_2).dot(avgPosition.normalize())) * cohesion;
				float seperationWeight = sgn(unit.direction.rotate(M_PI_2).dot(closestPosition.normalize())) * (1.f - (closestPosition.norm() / PERSONALSPACE)) * seperation;

				unit.nextDirection = unit.direction.rotate(UNITTURNING * ((alignWeight + cohesionWeight - seperationWeight) / (seperation + alignment + cohesion)) * dt);

			}
			else{
				unit.nextDirection = unit.direction;
			}
		}
		for(Unit& unit : units){
			if(unit.visible == false) continue;

			if(abs(unit.position.x) > MAXCENTERDIST){
				unit.nextDirection.x *= -1.f;
			}
			if(abs(unit.position.y) > MAXCENTERDIST){
				unit.nextDirection.y *= -1.f;
			}

			unit.direction = unit.nextDirection;
			unit.position += unit.direction * UNITSPEED * dt;

			if(unit.updateOccupiedSpace()) triggerSpace(unit.occupiedSpace);
		}

		//bool grids[10];

		for(int i = 0; i < 10; i++){
			bool gridStatus = gridPulseGens[i].process(dt);
			if(gridStatus) outputs[i].setVoltage(10.f);
			else outputs[i].setVoltage(0.f);
			//grids[i] = gridStatus;
		}

	}
};

struct BoidzDisplay : Widget{
	Boidz* module;
	float frame = 0.0;

	void drawLayer(const DrawArgs& args, int layer) override{
		if(layer == 1){

			if (module){
				float gridScale = (box.size.x / (module->MAXCENTERDIST * 2.f));
				float gridOffset = (box.size.x / 2.f);

				for(Boidz::Unit unit : module->units){
					if(unit.visible == false) continue;

					Vec unitPos = (unit.position * gridScale) + gridOffset;

					nvgFillColor(args.vg, nvgRGBf(1.f, 1.f, 1.f));
					nvgBeginPath(args.vg);
					nvgCircle(args.vg, unitPos.x, unitPos.y, 1.f * gridScale);
					nvgFill(args.vg);
					
				}
			}

			Widget::drawLayer(args, layer);
		}
	}
		
};

struct BoidzWidget : ModuleWidget {
	BoidzWidget(Boidz* module) {
		setModule(module);
		setPanel(createPanel(asset::plugin(pluginInstance, "res/boidz.svg")));

		addParam(createParamCentered<RoundBlackSnapKnob>(mm2px(Vec(25.4, 59.222)), module, Boidz::NUMBOIDS_PARAM));
		addParam(createParamCentered<Trimpot>(mm2px(Vec(10.16, 69.343)), module, Boidz::COHESIONATTN_PARAM));
		addParam(createParamCentered<Trimpot>(mm2px(Vec(20.32, 69.343)), module, Boidz::ALIGNMENTATTN_PARAM));
		addParam(createParamCentered<Trimpot>(mm2px(Vec(30.48, 69.343)), module, Boidz::SEPERATIONATTN_PARAM));

		addInput(createInputCentered<DarkPJ301MPort>(mm2px(Vec(15.24, 59.222)), module, Boidz::RESET_INPUT));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(10.16, 78.981)), module, Boidz::COHESIONIN_INPUT));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(20.32, 78.981)), module, Boidz::ALIGNMENTIN_INPUT));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(30.48, 78.981)), module, Boidz::SEPERATIONIN_INPUT));

		addOutput(createOutputCentered<DarkPJ301MPort>(mm2px(Vec(10.16, 91.556)), module, Boidz::TRIGA1_OUTPUT));
		addOutput(createOutputCentered<DarkPJ301MPort>(mm2px(Vec(20.32, 91.556)), module, Boidz::TRIGA2_OUTPUT));
		addOutput(createOutputCentered<DarkPJ301MPort>(mm2px(Vec(30.48, 91.556)), module, Boidz::TRIGA3_OUTPUT));
		addOutput(createOutputCentered<DarkPJ301MPort>(mm2px(Vec(10.16, 101.194)), module, Boidz::TRIGB1_OUTPUT));
		addOutput(createOutputCentered<DarkPJ301MPort>(mm2px(Vec(20.32, 101.194)), module, Boidz::TRIGB2_OUTPUT));
		addOutput(createOutputCentered<DarkPJ301MPort>(mm2px(Vec(30.48, 101.194)), module, Boidz::TRIGB3_OUTPUT));
		addOutput(createOutputCentered<DarkPJ301MPort>(mm2px(Vec(10.16, 110.831)), module, Boidz::TRIGC1_OUTPUT));
		addOutput(createOutputCentered<DarkPJ301MPort>(mm2px(Vec(20.32, 110.831)), module, Boidz::TRIGC2_OUTPUT));
		addOutput(createOutputCentered<DarkPJ301MPort>(mm2px(Vec(30.48, 110.831)), module, Boidz::TRIGC3_OUTPUT));
		addOutput(createOutputCentered<DarkPJ301MPort>(mm2px(Vec(20.32, 120.469)), module, Boidz::TRIGALL_OUTPUT));

		BoidzDisplay* boidzDisplay = createWidget<BoidzDisplay>(mm2px(Vec(1.045, 14.456)));
		boidzDisplay->setSize(mm2px(Vec(38.55, 38.55)));
		boidzDisplay->module = module;
		addChild(boidzDisplay);
	}
};


Model* modelBoidz = createModel<Boidz, BoidzWidget>("boidz");
