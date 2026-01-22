#include <iostream>
#include <bhc/bhc.hpp>

/*
std::ostream& operator<<(std::ostream& out, const bhc::rayPt<false>& x) {
    out << x.NumTopBnc << " "
        << x.NumBotBnc << " "
        << x.x.x << " " << x.x.y << " "
        << x.t.x << " " << x.t.y << " "
        << x.p.x << " " << x.p.y << " "
        << x.q.x << " " << x.q.y << " "
        << x.c << " "
        << x.Amp << " " << x.Phase << " "
        << x.tau;
    return out;
}
*/
std::ostream &operator<<(std::ostream &out, const bhc::cpxf &x)
{
    out << "(" << x.real() << ", " << x.imag() << ")";
    return out;
}

void OutputCallback(const char *message)
{
    std::cout << "Out: " << message << std::endl << std::flush;
}

void PrtCallback(const char *message) { std::cout << message << std::flush; }

int main()
{
    bhc::bhcParams<false> params;
    bhc::bhcOutputs<false, false> outputs;
    bhc::bhcInit init;
    init.FileRoot       = nullptr;
    init.outputCallback = OutputCallback;
    init.prtCallback    = PrtCallback;

    bhc::setup(init, params, outputs);
    bhc::echo(params);
    bhc::run(params, outputs);

    std::cout << "Field:\n";
    for(int iz = 0; iz < params.Pos->NRz; ++iz) {
        for(int ir = 0; ir < params.Pos->NRr; ++ir) {
            std::cout << outputs.uAllSources[iz * params.Pos->NRr + ir] << " ";
        }
        std::cout << "\n";
    }
    /*
    std::cout << "\n" << outputs.rayinfo->NRays << " rays:\n";
    for(int r=0; r<outputs.rayinfo->NRays; ++r){
        std::cout << "\nRay " << r
            << ", " << outputs.rayinfo->results[r].Nsteps
            << "steps, SrcDeclAngle = " << outputs.rayinfo->results[r].SrcDeclAngle
            << ":\n";
        for(int s=0; s<outputs.rayinfo->results[r].Nsteps; ++s){
            std::cout << outputs.rayinfo->results[r].ray[s] << "\n";
        }
    }
    */

    bhc::finalize(params, outputs);
    return 0;
}