import { Popover, Transition } from "@headlessui/react";
import { AdjustmentsIcon } from "@heroicons/react/solid";
import { useAtom } from "jotai";
import { Fragment } from "react";

import { sectionGraphPlotTypeAtom } from "@/data/settings";

import classNames from "@/util/classNames";

import { SectionLoadGraphPlotType } from "@/components/SectionLoadGraph";

const sectionGraphPlotTypes: Array<{
  plotType: SectionLoadGraphPlotType;
  label: string;
}> = [
  { plotType: "SimpleBox", label: "Vereinfachter Box-Plot" },
  { plotType: "Box", label: "Box-Plot" },
  { plotType: "Violin", label: "Violin-Plot" },
];

function SectionGraphPlotSettings() {
  const [selectedPlotType, setSelectedPlotType] = useAtom(
    sectionGraphPlotTypeAtom
  );

  return (
    <div className="bg-white p-7">
      Auslastungsgrafik:
      <div className="flex flex-col pl-3 pt-2 gap-2">
        {sectionGraphPlotTypes.map(({ plotType, label }) => (
          <label key={plotType} className="inline-flex items-center gap-2">
            <input
              type="radio"
              name="load-level"
              value={plotType}
              checked={selectedPlotType == plotType}
              onChange={() => setSelectedPlotType(plotType)}
            />
            {label}
          </label>
        ))}
      </div>
    </div>
  );
}

function Settings(): JSX.Element {
  return (
    <div className="absolute top-2 right-2">
      <Popover className="relative">
        {({ open }) => (
          <>
            <Popover.Button
              className={classNames(
                open ? "opacity-100" : "opacity-50",
                "p-2 mb-1 flex justify-center align-center bg-white text-black dark:bg-gray-600 dark:text-gray-100 rounded-full shadow-sm outline-0"
              )}
            >
              <AdjustmentsIcon className="w-4 h-4" aria-hidden="true" />
            </Popover.Button>
            <Transition
              as={Fragment}
              enter="transition ease-out duration-200"
              enterFrom="opacity-0 translate-y-1"
              enterTo="opacity-100 translate-y-0"
              leave="transition ease-in duration-150"
              leaveFrom="opacity-100 translate-y-0"
              leaveTo="opacity-0 translate-y-1"
            >
              <Popover.Panel className="absolute z-10 w-screen px-4 mt-1 right-0 sm:px-0 max-w-sm lg:max-w-md">
                <div className="overflow-hidden rounded-lg shadow-lg ring-1 ring-black ring-opacity-5">
                  <SectionGraphPlotSettings />
                </div>
              </Popover.Panel>
            </Transition>
          </>
        )}
      </Popover>
    </div>
  );
}

export default Settings;
