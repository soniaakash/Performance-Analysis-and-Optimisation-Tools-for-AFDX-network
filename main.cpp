/***************************************************************************
 * Performance Analysis and Optimisation Tools for AFDX network
 * Copyright (C) 2020  Aakash SONI
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#include <QCoreApplication>
#include <QCommandLineParser>
#include "NetworkCalculus/networkcalculus.h"

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    qStdOut() << "*** Performance Analysis and Optimisation Tools for AFDX network ***";

    NetworkCalculus nc; nc.init(AFDX::AFDX_SOURCE::FILE, SCHEDULING::WRR, MODE::WCTT, OPTIONS::OPTIMISED);

    qStdOut() << "Terminate! ";

    return 0;//a.exec();

}

